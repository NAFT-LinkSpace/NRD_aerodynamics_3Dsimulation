# 計算式について

[TOC]

## 1　推力履歴の読み込み
  外部ファイルから推力履歴を読み込む．
  推力履歴は燃焼試験データを使用する．
  open()を使いファイルを読み込み，以下の対応で値を格納
  - thrust_data_list：推力[N]
  - thrust_data_len：推力のデータ数
  - time_burn：燃焼時間[s]
  
  ```Python
  with open('./thrust.txt') as thrust_data_txt:
    thrust_data_list = [s.strip() for s in thrust_data_txt.readlines()]
    thrust_data_len = len(thrust_data_list)
    time_burn = (thrust_data_len + 1) * thrust_time_span
  ```
</br>

## 2　質量・慣性モーメントテンソルの計算
### 2.1　燃焼中の計算
  燃焼によって重量が変化するため，燃焼開始からの時間によって機体の重量を計算する．
  現在，燃焼中については以下の式を用いて線形近似している．
  ```math
  (質量)=\frac{(燃焼\textbf{後}質量)-(燃焼\textbf{前}質量)}{(燃焼時間)}\times(燃焼開始後経過時間)+(燃焼\textbf{前}質量)
  ```

  また，慣性モーメントテンソルも同様の関係式で計算している．
  重心・慣性モーメントテンソルはCADの値を使用する．

  ```python
  # 諸元代入 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  mass_before = 6.281    # 打ち上げ前機体重量[kg]
  mass_after = 6.201     # 燃焼終了後質量[kg]
  Inertia_before = np.array([[0.01072419406, 0.00016134245, 0.00014908097],
                            [0.00016134245, 0.86274073026, 0.00001128463],
                            [0.00014408097, 0.00001128463, 0.86262115772]], dtype = np.float64)   
    # 打ち上げ前機体慣性モーメントテンソル[kg*m*m]

  Inertia_after = np.array([[0.01072419406, 0.00016134245, 0.00014908097],
                            [0.00016134245, 0.86274073026, 0.00001128463],
                            [0.00014408097, 0.00001128463, 0.86262115772]], dtype = np.float64)   
    # 燃焼終了後機体慣性モーメントテンソル[kg*m*m]


  # 変数宣言 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  mass = mass_before        # 機体質量[kg]
  Inertia = Inertia_before  # 機体慣性モーメント[kg*m*m]
  
  # 計算 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if time < time_burn:  # 燃焼中の演算
      mass    = ( (mass_after - mass_before)       / time_burn ) * time + mass_before
      Inertia = ( (Inertia_after - Inertia_before) / time_burn ) * time + Inertia_before
      np.nan_to_num(Inertia, copy=False)  # nan対策
  ```

### 2.2　燃焼後の計算
  燃焼後は質量や慣性モーメントテンソルは変化しないため，燃焼終了時の値を使う．

  ```python
    if time < time_burn:  # 燃焼中の演算
      # (略)
    else: # 燃焼していない時の演算
      mass = mass_after
      Inertia = Inertia_after
      thrust = 0.0
  ```
</br>

## 3　推力の計算
### 3.1　推力データの選択
  演算の時間の刻み幅と推力履歴の時間の刻み幅が異なることに対応するため，時間に応じて使用するデータを選択する．
  推力履歴のデータが燃焼開始何秒後かを求めるために以下の式を用いる．

  ```math
  (推力履歴上の経過時間) =\\\
  (推力履歴の時間の刻み幅)\times(参照している推力データリストのインデックス)
  ```

  求めた秒数とシミュレーション上での燃焼開始後の時間を比較し，シミュレーション上での時間の方が進んでいた場合はインデックスを1増やして次のデータを参照するようにする．
  また，燃焼終了後は推力の変数に0.0を代入する．

  ```python
  # time_burn:燃焼時間
  # thrust_data_number:推力履歴のインデックス

  if time < time_burn:  # 燃焼中の演算
    if time > thrust_time_span * (thrust_data_number + 1) : 
        thrust_data_number = thrust_data_number + 1 
  else: # 燃焼していない時の演算
    thrust = 0.0
  ```
  

### 3.2　推力データの近似
  推力履歴の時間刻み幅よりシミュレーションの時間刻み幅の方が短いことが多い．
  そのため，推力のデータを線形近似で補完する．
  推力の単位は[N]．
  以下に推力履歴の時間刻み幅を$\Delta t$として補完の式を示す．

  ```math
　(T[s]後の推力)=(t[s]後の推力)+\frac{((t+\Delta t)[s]後の推力)-(t[s]後の推力)}{\Delta t[s]}\times(T[s]- (推力履歴上の経過時間))
  ```

  ```python
  # time_burn:燃焼時間
  # thrust_data_number:推力履歴のインデックス

  if time < time_burn:  # 燃焼中の演算
      if thrust_data_number + 2 <= thrust_data_len :
        thrust = np.double(thrust_data_list[thrust_data_number]) + (np.double(thrust_data_list[thrust_data_number + 1]) - np.double(thrust_data_list[thrust_data_number])) * (time - thrust_time_span * thrust_data_number) / thrust_time_span 
  ```

##　4　空力特性の計算
