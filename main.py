#                                                        ....J+ggggg&J....
#                                                  ..&MMMMMMMMMMMMMMMMMMMMMMNg,.
#                                              ..MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMN..
#                                           .JMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMNa.
#                                         .MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMN,
#                           ...(+uwwwA&+JMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMx
#                       .JXyyyyyyyyyyyyyyyyyyWMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM,
#                   ..dyyyyyyyyyyyyyyyyyyyyZyyyyyWMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMN,
#                 .JyyyyyyZyyZyZyZyZyZyZyZyyZyZyZyZyMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMe
#               .dyyyZyyZyyZyyZyyyZyyyZyyyZZyZZZyZZyZyMMNMMMMMMMMMMMMMMMNMNMNMNMNMMMMMMMMMMMMMMMb
#              .yyyZyyyZyyyyZy$          (               (             ,!        ( ?TMMMMMMMMMMMMR
#             ?TTTWyZyyyZyZyyyk..........(               (             ,!        (     ?WMMMMMMMMMb
#        ...........(7WyyZyyZy$`         (               (             ,!        (  ..JMMMMMMMMMMMM[
#     ....~..~....~....?UyyZyyk..........J(..............J(............(+........+JMMMMMMMMMMMMMMMMN.
#    .....~..~.~....~....?yyyyy~    .yyyyk     yyyyy0      jMMMMM)            `              .MMMMMMb
#   .~.~........~..~..~...jyZyZ~     (yyyk     yyyyy\      .MMMMM)            `              .MMMMMMN
#  .....~.~..~....~....~.~.XyZy~      (yyk     yyZyX`       zMMMM)     .......`              .MMMMMMM|
#  ..~...~..~..~....~...~..(yyy~       jyk     yyZyr        .MMMM)    ,MMMMMMMNNNNN_    .NNNNMMMMMMMM]
#  ...~........~..~..~.....(yZZ~        Tk     yyyX`   .;    dMMM)    .MMMMMMMMMMMM:    -MMMMMMMMMMMMF
#  ..~..~.~.~...~..~..~.~..dyyy~         U  `  yyZr    ,$    ,MMM)  `        MMMMMM:    -MMMMMMMMMMMMF
#   .........~...........~(yyZy~    -          yyy`    dy.    MMM)           MMMMMM:    -MMMMMMMMMMMM]
#    ..~..~....~..~.~..~.(yyyyZ~    j,         yyr    .UU\    ,MM)    .......MMMMMM:    -MMMMMMMMMMMM:
#     _.~..~..~..~....~(dyyZyyy~    jy,  `  `  yy!             MM)    .MMMMMMMMMMMM:    -MMMMMMMMMMM#
#        _...~.....~((dyyyZyZyZ~    jyk.       y$              -M)    .MMMMMMMMMMMM:    -MMMMMMMMMMMt
#             +++uXyyyZyyZyyyZy~    jyyk.      y:   ` .....  `  M)   `.MMMMMMMMMMMM:    -MMMMMMMMMM#
#              4yyyZyZyyZyyyyyy~    jyyZn      f     dyydMM.    ()    .MMMMMMMMMMMM:    -MMMMMMMMMM!
#               ?WyyyyyZyyZyZyZ~  ` Jyyyyl  `  : `  .yQMMMM] `   \ `  .MMMMMMMMMMMM:    -MMMMMMMMM^
#                 ?WyZyyyZyyyZyyyyyyyyZyyyyyyyyyyyyyQMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM3
#                   (4yyZyyZyyyyZyyyyyyZyyyyyyyyyWQMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM^
#                      ?7WyyZyyZyyZyZyyyZyZyyyk7MUMMMMMMMMMMMMMYM"MM#WMMMMMMMMBMHMMMMMMMMMMMMM@`
#                          _?7TUXyyZyZyWQQgNMMN..,-`%,,.,[Fa(,M]d!),<(-(':%.;<]l./$dMMMMMMMMM3
#                                      TMMMMMMNJNJJgF>-&J@.N(JMNJJaNgJmMm(mdaJmN(@.MMMMMMMMD
#                                       .TMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMD
#                                          7MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM#=
#                                             TMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM#"
#                                                ?TMMMMMMMMMMMMMMMMMMMMMMMMMMMMM"^
#                                                     ?TWMMMMMMMMMMMMMMMMM""^

# ライブラリのインストール~~~~~~~~~~~~~~~~~~
from cmath import asin, atan, pi
import datetime
import csv
import math
from math import cos, sin, acos
from sre_parse import HEXDIGITS
import numpy as np
from numpy import linalg, true_divide
from scipy import interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
dtdt = datetime.datetime.now()
TIME_NOW = dtdt.strftime('%Y-%m%d-%H%M%S')
# 要検討resultは作るかすでにあるとエラーになる
os.mkdir('./result2_check/' + str(TIME_NOW))

# fig = plt.figure()
# ax = Axes3D(fig)
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# ax.set_zlabel("z")

# plt.pyplot.scatter()
# x1 = list()
# y1 = list()
# z1 = list()
# u1 = list()
# v1 = list()
# w1 = list()

# x2 = list()
# y2 = list()
# z2 = list()
# u2 = list()
# v2 = list()
# w2 = list()

# x3 = list()
# y3 = list()
# z3 = list()
# u3 = list()
# v3 = list()
# w3 = list()
# ax.quiver(x1, y1, z1, u1, v1, w1, color = "red")
# ax.quiver(x2, y2, z2, u2, v2, w2, color = "green")
# ax.quiver(x3, y3, z3, u3, v3, w3, color = "blue")
# ax.quiver(x4, y4, z4, u4, v4, w4, color = "purple")
# ax.quiver(x5, y5, z5, u5, v5, w5, color = "orange")
# ax.quiver(x6, y6, z6, u6, v6, w6, color = "aqua")

# max_range = max(np.array([max(x1)-min(x1), max(y1)-min(y1), max(z1)-min(z1)])) * 0.5
# mid_x = (max(x1)+min(x1)) * 0.5
# mid_y = (max(y1)+min(y1)) * 0.5
# mid_z = (max(z1)+min(z1)) * 0.5
# ax.set_xlim(mid_x - max_range, mid_x + max_range)
# ax.set_ylim(mid_y - max_range, mid_y + max_range)
# ax.set_zlim(mid_z - max_range, mid_z + max_range)

# plt.show()
# print("落下位置は(y, z)=("+str(X[0, 1, 0])+', '+str(X[0, 2,0])+')')

# メイン処理~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


def main():

    # 変数を宣言
    initialization()

    # 各風向風速で計算を回すための処理
    calc_eachcase()

    # 終了処理
    finalization()


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def initialization():

    global MASS_BEFORE, MASS_AFTER, INERTIA_BEFORE, INERTIA_AFTER, CL_ALPHA_RAD, CD_0, CD_ALPHA_RAD, CN_ALPHA_RAD, CDS_PARA, CDS_PARA_REEFING, V_PARA, V_PARA_REEFING, ROLL_DUMP, PITCH_DUMP, BODY_DIAMETER, POSITION_CG_END_BEFORE, POSITION_CG_END_AFTER, BODY_LENGTH, POSITION_LAUNCH_LAG_END, POSITION_CP_ALPHAMAX_END, POSITION_CP_ALPHA0_END, RECOVERY_TIMER, REEFING_TIMER
    global TIME_SPAN, LAUNCH_ANGLE_DEG, LAUNCHER_LENGTH, LAUNCHER_LONGITUDE, LAUNCHER_LATITUDE, ALPHA_MAX_DEG, THRUST_TIME_SPAN, HEIGHT_STD_DENSITY, PRESSURE_STD, TENPERATURE_STD, HEIGHT_STD_WIND, WIND_POW, WIND_NUM, WIND_STD_INIT, WIND_STD_INTERVAL, AZIMUTH_NUM, WIND_ANGLE_INIT_DEG, LAUNCHER_AZUMITH_DEG, RECOVERY_STATUS, REEFING_STATUS, WIND_STATUS, AERODYNAMICS_STATUS, ATTENUATION_STATUS
    global FINISH_HEIGHT, FINISH_JUDGMENT_START_TIME, MAX_SIMULATION_ROOP, G, R, ALPHA_MAX_RAD, LAUNCH_ANGLE_RAD, ROCKET_AREA, thrust_data_list, TIME_BURN, landing_result, wind_std, wind_angle, cds_para_now, v_para_now

    # 機体諸元~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # 定数は全て大文字で書く慣例があるので
    # ******************要検討*****************0にしてはいけない，変数型によってエラーを返せるといいね
    MASS_BEFORE = 5.738   # 打ち上げ前機体質量[kg]　#6.120@審査書諸元表：元シミュ5.73752
    MASS_AFTER = 5.304    # 燃焼終了後質量[kg]   #6.034@審査書諸元表：元シミュ5.30352

    # 打ち上げ前機体慣性モーメントテンソル[kg*m*m]
    INERTIA_BEFORE = np.array([[0.01107, 0.0, 0.0],
                               [0.0, 0.6444, 0.0],
                               [0.0, 0.0, 0.6444]], dtype=np.float32)

    # INERTIA_BEFORE = np.array([[0.64438622, 0.0, 0.0],
    #                         [ 0.0, 0.01107, 0.0],
    #                         [ 0.0, 0.0, 0.01107]], dtype = np.float32)

    # 燃焼終了後機体慣性モーメントテンソル[kg*m*m]
    INERTIA_AFTER = np.array([[0.01107, 0.0, 0.0],
                              [0.0, 0.6444, 0.0],
                              [0.0, 0.0, 0.6444]], dtype=np.float32)

    # INERTIA_AFTER = np.array([[0.64438622, 0.0, 0.0],
    #                         [ 0.0, 0.01107, 0.0],
    #                         [ 0.0, 0.0, 0.01107]], dtype = np.float32)

    CL_ALPHA_RAD = 5.093                             # 揚力傾斜[/rad]
    CD_0 = 0.4401                                # 迎角0の時の抗力係数かつ軸力係数[]
    CD_ALPHA_RAD = 0.0                       # 迎角二乗の比例定数[/rad/rad]

    CN_ALPHA_RAD = 5.093                                 # 法線力傾斜[/rad]

    CDS_PARA = 0.825           # パラシュートの抗力係数×投影面積[m*m]
    CDS_PARA_REEFING = 0.416   # リーフィング時のパラシュートの抗力係数×投影面積[m*m]

    V_PARA = 10.4           # パラシュートの終端速度[m/s]
    V_PARA_REEFING = 14.3   # リーフィング時のパラシュートの終端速度[m/s]

    # 元々-61.81だったのに第二版諸元表では-53.91
    ROLL_DUMP = -61.81        # ロールダンピング係数[]
    PITCH_DUMP = -5.509        # ピッチ(ヨー)ダンピング係数[]　20230126yamaoka modified

    BODY_DIAMETER = 0.114  # 機体直径[m]

    BODY_LENGTH = 1.279               # 機体全長(ピトー管あるなら含む)[m]

    # 計算めんどいので結果が間違っていたら修正しにくる
    POSITION_CG_END_BEFORE = 0.54371
    POSITION_CG_END_AFTER = 0.55520

    # プログラム上では0.36036だった0.36でもどちらでも問題なさそう
    POSITION_CP_ALPHAMAX_END = 0.36036  # 機体後端から測った迎角最大時の圧力中心の距離[m]
    POSITION_CP_ALPHA0_END = 0.36036     # 機体後端から測った迎角0の時の圧力中心の距離[m]

    POSITION_LAUNCH_LAG_END = 0.002   # 機体後端からエンドランチラグまでの距離[m]

    RECOVERY_TIMER = 12.0       # タイマー開傘時刻[s]
    REEFING_TIMER = 25.0        # リーフィング開傘時刻[s]

    # シミュ設定~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    TIME_SPAN = 0.01                    # 時間刻み幅[s]

    # ランチャー角[deg](鉛直となす角なので注意，慣例的な方の定義で入力できるようにしたいね)
    LAUNCH_ANGLE_DEG = 80.0
    LAUNCHER_LENGTH = 5.0               # ランチャーレール長[m]
    LAUNCHER_LONGITUDE = 34.736139      # ランチャーの緯度[deg：十進法]
    LAUNCHER_LATITUDE = 139.421333      # ランチャーの経度[deg：十進法]

    ALPHA_MAX_DEG = 10.0                # 想定最大迎角[°](0にしてはいけない)

    THRUST_TIME_SPAN = 0.01             # 推力履歴の時間刻み幅[s]

    # 空気密度について
    # 参考：http://zakii.la.coocan.jp/physics/32_density.htm（ロケットが11[km]を超えない想定で数式を使っている）
    # 怪しい↓
    HEIGHT_STD_DENSITY = 1.293          # 空気密度を考える際の基準高度[m]
    PRESSURE_STD = 101325.0             # 基準高度の気圧[Pa]
    TENPERATURE_STD = 15.0 + 273.15     # 基準高度の気温[K](ケルビンなので注意；決して0にするな)

    # べき乗則風
    HEIGHT_STD_WIND = 2.0               # べき法則風を考える際の基準高度[m](決して0にするな)
    WIND_POW = 1.0/6.0                  # べき法則の指数[](一様風にしたければ0.0にすること)

    WIND_NUM = 1                        # 調べたい風速の数[](自然数を入れること)
    WIND_STD_INIT = 3.0                 # 基準風速[m/s](0.0とすれば理想フライトのデータが取れる)
    WIND_STD_INTERVAL = 1.0             # 調べたい風速の間隔[m/s]

    AZIMUTH_NUM = 1                     # 調べたい方位の数（自然数）
    # 風向の初期値：上空から見てランチャを0[°]とし""反時計回りを正""にしてどの方向から風が吹いてくるか
    WIND_ANGLE_INIT_DEG = 0.0
    # 要確認
    LAUNCHER_AZUMITH_DEG = 90.0 - 7.53  # 描画したい地図の北に対するランチャ方位角[deg]

    # 開傘を行うか弾道落下か，[0, 1, 2] = [弾道，頂点開傘，タイマー開傘]
    RECOVERY_STATUS = 1
    REEFING_STATUS = 1                  # リーフィングを行うか，[0，1] = [リーフィングなし，あり]
    # *******************実装しろ*****************************
    WIND_STATUS = 1                     # 風速モデル[1, 2] = [冪法則風，統計風]
    AERODYNAMICS_STATUS = 1             # 空力モデル[0, 1] = [揚抗力，軸法線力]
    # 元々0だった(多分元々書いてあるコメントが逆0=しない1=する)
    ATTENUATION_STATUS = 0              # 減衰モーメントを考慮するか[0, 1] = [する，しない]

    # 初期設定~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    FINISH_HEIGHT = 0.0                 # 演算終了判定高度[m]
    FINISH_JUDGMENT_START_TIME = 5.0    # 演算終了判定開始時刻[s]
    # 一つの軌道につきループを回す最大数[](エラーが起きて無限ループが始まるとプログラムが終わらなくなるので)
    MAX_SIMULATION_ROOP = 30000

    G = 9.80665                         # 重力加速度[m/s/s](物理定数ゆえにいじるな)
    R = 287.0                           # 大気の気体定数[J/kg/K](物理定数？ゆえにいじるな)

    ALPHA_MAX_RAD = ALPHA_MAX_DEG * np.pi / 180.0        # 想定最大迎角[rad]
    LAUNCH_ANGLE_RAD = LAUNCH_ANGLE_DEG * np.pi / 180.0  # ランチャー角[rad]
    ROCKET_AREA = 0.25 * BODY_DIAMETER * \
        BODY_DIAMETER * np.pi           # ロケット断面積[m*m]

    # 同階層内のthrust.txtを読み込む
    # *****************複数のデータから回せるといいね**********************
    with open('./thrust/2023-1-22 Deta0added.txt') as thrust_data_txt:
        thrust_data_list = [s.strip()
                            for s in thrust_data_txt.readlines()]     # 推力履歴を配列に収める
        # データの個数[]
        THRUST_DATA_LEN = len(thrust_data_list)
        TIME_BURN = THRUST_DATA_LEN * \
            THRUST_TIME_SPAN                          # 燃焼時間[s]が求まる

    # 各風速，各方位毎に各データを出力する変数を宣言
    landing_result = np.zeros((WIND_NUM, AZIMUTH_NUM, 4, 3), dtype=np.float32)

    wind_std = WIND_STD_INIT    # 基準風速の初期値[m/s]
    wind_angle = WIND_ANGLE_INIT_DEG    # 風向の初期値[deg]

    if (REEFING_STATUS == 0):
        cds_para_now = CDS_PARA
        v_para_now = V_PARA
    elif (REEFING_STATUS == 1):
        cds_para_now = CDS_PARA_REEFING
        v_para_now = V_PARA_REEFING


def calc_eachcase():

    # 基準風速を変えながら
    wind_roop = 0
    while (wind_roop < WIND_NUM):

        # 風向を変えながら
        azm_roop = 0
        global wind_angle
        wind_angle = WIND_ANGLE_INIT_DEG

        while (azm_roop < AZIMUTH_NUM):

            with open('./result2_check/' + TIME_NOW + '/history_' + str(int(wind_angle)) + 'deg_wind' + str(wind_roop) + '.csv', 'w', newline='') as file:
                global filecsv
                filecsv = csv.writer(file)
                # 出力の単位間違えてませんか？
                filecsv.writerow(["time[s]",
                                  "x_CG[m]",
                                  "y_CG[m]",
                                  "z_CG[m]",
                                  "vx_CG[m/s]",
                                  "vy_CG[m/s]",
                                  "vz_CG[m/s]",
                                  "ang_vx_B[rad/s]",
                                  "ang_vy_B[rad/s]",
                                  "ang_vz_B[rad/s]",
                                  "q0[]",
                                  "q1[]",
                                  "q2[]",
                                  "q3[]",
                                  "q_norm[]",
                                  "trans_mat_norm[]",
                                  "mass[kg]",
                                  "thrust[N]",
                                  "position_CG_end[m]",
                                  "vx_B[m/s]",
                                  "vy_B[m/s]",
                                  "vz_B[m/s]",
                                  "v_norm[m/s]",
                                  "wind_x_B[m/s]",
                                  "wind_y_B[m/s]",
                                  "wind_z_B[m/s]",
                                  "wind_norm[m/s]",
                                  "airflow_vx_B[m/s]",
                                  "airflow_vy_B[m/s]",
                                  "airflow_vz_B[m/s]",
                                  "airflow_vnorm[m/s]",
                                  "airdensity[kg/m/m/m]",
                                  "dp[N/m/m]",
                                  "alpha[deg]",
                                  "Cl[]",
                                  "Cn[]",
                                  "Cd[]",
                                  "-Cp_position_BODY_x[m]",
                                  "Fst",
                                  "force_x_Lift_B[N]",
                                  "force_y_Lift_B[N]",
                                  "force_z_Lift_B[N]",
                                  "force__norm_Lift_B[N]",
                                  "force_x_Drag_B[N]",
                                  "force_y_Drag_B[N]",
                                  "force_z_Drag_B[N]",
                                  "force_norm_Drag_B[N]",
                                  "torque_x_Lift_B[N*m]",
                                  "torque_y_Lift_B[N*m]",
                                  "torque_z_Lift_B[N*m]",
                                  "torque_norm_Lift_B[N*m]",
                                  "torque_x_Drag_B[N*m]",
                                  "torque_y_Drag_B[N*m]",
                                  "torque_z_Drag_B[N*m]",
                                  "torque_norm_Drag_B[N*m]",
                                  "force_x_reaction_B[N]",
                                  "force_y_reaction_B[N]",
                                  "force_z_reaction_B[N]",
                                  "torque_x_reaction_B[N*m]",
                                  "torque_y_reaction_B[N*m]",
                                  "torque_z_reaction_B[N*m]",
                                  "accel_BODY_x[m/s/s]",
                                  "accel_BODY_y[m/s/s]",
                                  "accel_BODY_z[m/s/s]",
                                  "Downrange[m]",
                                  "Flight_status",
                                  "cds_para_now[m*m]",
                                  "torque_attenuation_BODY_x[N*m]",
                                  "torque_attenuation_BODY_y[N*m]",
                                  "torque_attenuation_BODY_z[N*m]",
                                  "v_para_now[m/s]",
                                  "wind_x_B[m/s]",
                                  "wind_y_B[m/s]",
                                  "wind_z_B[m/s]"])

                # 打ち上げてから着地するまでの一回分の計算
                landing_result[wind_roop, azm_roop] = calc_main()

            wind_angle = (wind_angle + 360.0 / AZIMUTH_NUM) % 360
            azm_roop = azm_roop + 1

        global wind_std
        wind_std = wind_std + WIND_STD_INTERVAL
        wind_roop = wind_roop + 1

# 一度飛ばして着陸するまでの一連の計算~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


def calc_main():

    time = 0.0          # シュミレーション時間
    roop = 0            # 演算回数
    flight_status = 1   # 飛行状態[0,1,2,3]=[計算しない,ランチャー中,弾道中,開傘中]

    # Xは13成分の配列[機体重心のX座標,Y座標,Z座標,重心速度のX成分,Y成分,Z成分,角速度のX成分,Y成分,Z成分,クォータニオンq0成分,q1,q2,q3]というようにまとめた
    # クォータニオンの初期値はランチャー角を考慮した姿勢変換成分を設定している
    # X = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -sin(LAUNCH_ANGLE_RAD / 2), 0.0, cos(LAUNCH_ANGLE_RAD / 2)], dtype=np.float32)
    X = np.zeros(13, dtype=np.float32)
    X[0] = POSITION_CG_END_BEFORE * sin(LAUNCH_ANGLE_RAD)
    X[2] = POSITION_CG_END_BEFORE * cos(LAUNCH_ANGLE_RAD)
    X[9:13] = [0.0, -sin((np.pi / 2 - LAUNCH_ANGLE_RAD) / 2),
               0.0, cos((np.pi / 2 - LAUNCH_ANGLE_RAD) / 2)]

    # 四次ルンゲクッタ法のための係数
    K = \
        Ka = \
        Kb = \
        Kc = \
        Kd = np.zeros(13, dtype=np.float32)

    # 色んな計算結果が収まる配列を宣言
    # [落下位置(y,z)座標，レンジ]
    # [最高高度[m]，その時点の対気速度の大きさ[m/s]，その時刻[s]]
    # [最大動圧[m]，その時点の対気速度の大きさ[m/s]，その時刻[s]]
    # [最大機軸加速度[m]，その時点の対気速度の大きさ[m/s]，その時刻[s]]
    landing_point = np.zeros(3, dtype=np.float32)
    max_height = np.zeros(3, dtype=np.float32)
    max_dp = np.zeros(3, dtype=np.float32)
    max_accel = np.zeros(3, dtype=np.float32)
    # landing_point =  \
    # max_height = \
    # max_dp = \
    # max_accel = np.zeros(3, dtype=np.float32)
    # のようにやるとlanding_resultが全てlanding_pointの値になってしまう
    # ******************要検討****************************
    global cds_para_now, v_para_now
    if (REEFING_STATUS == 0):
        cds_para_now = CDS_PARA
        v_para_now = V_PARA
    elif (REEFING_STATUS == 1):
        cds_para_now = CDS_PARA_REEFING
        v_para_now = V_PARA_REEFING

    # flight_statusが0でない限りループを続けろ
    while not (flight_status == 0):
        state_quantity_now = time_develop_coefficient(time, flight_status, X)

        filecsv.writerow(state_quantity_now[13:87])  # 四次ルンゲクッタ法による更新

        Ka = state_quantity_now[0:13]
        Kb = time_develop_coefficient(
            time + TIME_SPAN * 0.5, flight_status, X + 0.5 * Ka * TIME_SPAN)[0:13]
        Kc = time_develop_coefficient(
            time + TIME_SPAN * 0.5, flight_status, X + 0.5 * Kb * TIME_SPAN)[0:13]
        Kd = time_develop_coefficient(
            time + TIME_SPAN, flight_status, X + Kc * TIME_SPAN)[0:13]
        K = (Ka + 2.0 * Kb + 2.0 * Kc + Kd) / 6.0
        X = X + K * TIME_SPAN

        # クォータニオンのノルムを1に矯正(これやらないと色々な物理量がバカみたいに発散する)
        X[9:13] = X[9:13] / np.linalg.norm(X[9:13])

        # 変換行列(詳細は同じものが後で出てくるので後述)
        trans_mat_main = np.array([[X[9] ** 2 - X[10] ** 2 - X[11] ** 2 + X[12] ** 2,    2 * (X[9]*X[10] + X[11]*X[12]),                         2 * (X[9]*X[11] - X[10]*X[12])],
                                   [2 * (X[9]*X[10] - X[11]*X[12]),                        -1 * X[9] ** 2 +
                                    X[10] ** 2 - X[11] ** 2 + X[12] ** 2,   2 * (X[10]*X[11] + X[9]*X[12])],
                                   [2 * (X[9]*X[11] + X[10]*X[12]),                        2 * (X[10]*X[11] - X[9]*X[12]),                         -1 * X[9] ** 2 - X[10] ** 2 + X[11] ** 2 + X[12] ** 2]])
        np.nan_to_num(trans_mat_main, copy=False)

        # 時間をすすめる
        time = time + TIME_SPAN
        roop = roop + 1
        # (補足)ここである時刻とその物理量が全て出揃うので以下様々な判定を行う（つまり構成上time = 0[s]では判定を行えないようになっている）

        # 最大高度判定
        vertex_pass_check = 1   # 少し後のパラ開傘判定に用いる

        if (max_height[0] < X[0]):
            max_height[0] = X[0]
            max_height[1] = np.linalg.norm(
                np.dot(trans_mat_main, wind_calc(X[0]) - [[X[3]], [X[4]], [X[5]]]))
            max_height[2] = time

            vertex_pass_check = 0   # 最大高度が更新されるということはまだ頂点通過してないよねということ

        # 最大動圧判定
        if (max_dp[0] < 0.5 * air_density_calc(X[0]) * (np.linalg.norm(np.dot(trans_mat_main, wind_calc(X[0]) - [[X[3]], [X[4]], [X[5]]])) ** 2) * ROCKET_AREA):
            max_dp[0] = 0.5 * air_density_calc(X[0]) * (np.linalg.norm(np.dot(
                trans_mat_main, wind_calc(X[0]) - [[X[3]], [X[4]], [X[5]]])) ** 2) * ROCKET_AREA
            max_dp[1] = X[0]
            max_dp[2] = time

        # 最大加速度判定(これが動いたら配列を抜き取る書き方に統一しよう)
        if (max_accel[0] < np.dot(trans_mat_main, ((Ka + 2.0 * Kb + 2.0 * Kc + Kd) / 6.0 / TIME_SPAN)[3:6])[0]):
            max_accel[0] = np.dot(
                trans_mat_main, ((Ka + 2.0 * Kb + 2.0 * Kc + Kd) / 6.0 / TIME_SPAN)[3:6])[0]
            max_accel[1] = X[0]
            max_accel[2] = time

        # ランチクリア判定（要検討）
        if (flight_status == 1) & (X[0] > (LAUNCHER_LENGTH + state_quantity_now[31] - POSITION_LAUNCH_LAG_END) * cos(np.pi / 2 - LAUNCH_ANGLE_RAD)):
            flight_status = 2
            print(time)
            print(state_quantity_now[35])

        # パラ開傘判定
        if (flight_status == 2) & (((RECOVERY_STATUS == 1) & (vertex_pass_check == 1)) or ((RECOVERY_STATUS == 2) & (time > RECOVERY_TIMER))):
            flight_status = 3

        if ((time >= REEFING_TIMER) & (REEFING_STATUS == 1)):
            cds_para_now = CDS_PARA
            v_para_now = V_PARA

        # 終了判定（計算が発散したら着地座標もクソもない〜要検討）
        if ((X[0] <= FINISH_HEIGHT) and (time >= FINISH_JUDGMENT_START_TIME)) or (roop >= MAX_SIMULATION_ROOP):
            flight_status = 0
            landing_point[0] = X[1]
            landing_point[1] = X[2]
            landing_point[2] = np.linalg.norm([X[1], X[2]])

    return np.array([landing_point, max_height, max_dp, max_accel])    # 結果を返す

    # 三次元グラフ描画用のあれこれ
    # if (roop % 1000 == 0):
    # x1 = x1 + [np.double(X[0])]
    # y1 = y1 + [np.double(X[1])]
    # z1 = z1 + [np.double(X[2])]
    # u1 = u1 + [20*np.dot(trans_mat_main.T,np.array([[1.0],[0.0],[0.0]],dtype = np.float32))[0,0]]
    # v1 = v1 + [20*np.dot(trans_mat_main.T,np.array([[1.0],[0.0],[0.0]],dtype = np.float32))[1,0]]
    # w1 = w1 + [20*np.dot(trans_mat_main.T,np.array([[1.0],[0.0],[0.0]],dtype = np.float32))[2,0]]

    # x2 = x2 + [np.double(X[0])]
    # y2 = y2 + [np.double(X[1])]
    # z2 = z2 + [np.double(X[2])]
    # u2 = u2 + [20*np.dot(trans_mat_main.T,np.array([[0.0],[1.0],[0.0]],dtype = np.float32))[0,0]]
    # v2 = v2 + [20*np.dot(trans_mat_main.T,np.array([[0.0],[1.0],[0.0]],dtype = np.float32))[1,0]]
    # w2 = w2 + [20*np.dot(trans_mat_main.T,np.array([[0.0],[1.0],[0.0]],dtype = np.float32))[2,0]]

    # x3 = x3 + [np.double(X[0])]
    # y3 = y3 + [np.double(X[1])]
    # z3 = z3 + [np.double(X[2])]
    # u3 = u3 + [20*np.dot(trans_mat_main.T,np.array([[0.0],[0.0],[1.0]],dtype = np.float32))[0,0]]
    # v3 = v3 + [20*np.dot(trans_mat_main.T,np.array([[0.0],[0.0],[1.0]],dtype = np.float32))[1,0]]
    # w3 = w3 + [20*np.dot(trans_mat_main.T,np.array([[0.0],[0.0],[1.0]],dtype = np.float32))[2,0]]


def wind_calc(height_arg_w):
    # その高度におけ地上風ベクトルを返す関数

    if WIND_STATUS == 1:
        if (height_arg_w <= 0.0):
            wind_velocity_norm = wind_std
        else:
            wind_velocity_norm = wind_std * \
                math.exp(WIND_POW * math.log(abs(height_arg_w / HEIGHT_STD_WIND)))
        return np.array([[0.0], [-wind_velocity_norm * sin(wind_angle * np.pi / 180.0)], [-wind_velocity_norm * cos(wind_angle * np.pi / 180.0)]], dtype=np.float32)

    elif WIND_STATUS == 2:
        print('未実装')
        return np.zeros(3, dtype=np.float32)
        # 統計風の実装の必要あり（thrustの処理と類似するものになるだろう）


def air_density_calc(height_arg_d):
    # その高度における空気密度を返す関数
    return PRESSURE_STD * math.exp((G / 0.0065 / R) * math.log(1.0 - 0.0065 * (height_arg_d - HEIGHT_STD_DENSITY) / TENPERATURE_STD)) / R / (TENPERATURE_STD - 0.0065 * (height_arg_d - HEIGHT_STD_DENSITY))


def time_develop_coefficient(t, sts, X_arg):
    # *************要検討***************[0]か[0,1]表記か統一すること
    CG_G = np.zeros((3, 1), dtype=np.float32)
    CG_G[0, 0] = X_arg[0]
    CG_G[1, 0] = X_arg[1]
    CG_G[2, 0] = X_arg[2]
    np.nan_to_num(CG_G, copy=False)

    CGv_G = np.zeros((3, 1), dtype=np.float32)
    CGv_G[0, 0] = X_arg[3]
    CGv_G[1, 0] = X_arg[4]
    CGv_G[2, 0] = X_arg[5]
    np.nan_to_num(CGv_G, copy=False)

    angle_velocity = np.zeros((3, 1), dtype=np.float32)
    angle_velocity[0, 0] = X_arg[6]
    angle_velocity[1, 0] = X_arg[7]
    angle_velocity[2, 0] = X_arg[8]
    np.nan_to_num(angle_velocity, copy=False)

    Qtn = np.zeros((4, 1), dtype=np.float32)
    Qtn[0, 0] = X_arg[9]
    Qtn[1, 0] = X_arg[10]
    Qtn[2, 0] = X_arg[11]
    Qtn[3, 0] = X_arg[12]
    np.nan_to_num(Qtn, copy=False)

    Qtn_developer = np.zeros((4, 1), dtype=np.float32)
    velocity_wind_GND = \
    velocity_airflow_BODY = \
    force_gravity_GND = \
    force_thrust_BODY = \
    force_lift_BODY = \
    force_drag_BODY = \
    force_reaction_BODY = \
    accel_GND = \
    torque_thrust_BODY = \
    torque_lift_BODY = \
    torque_drag_BODY = \
    torque_reaction_BODY = \
    angle_accel_BODY = \
    torque_attenuation_BODY = np.zeros((3, 1), dtype=np.float32)

    # 質量と慣性モーメントと推力と重心位置の更新
    if t <= TIME_BURN:  # 燃焼中の演算
        mass = ((MASS_AFTER - MASS_BEFORE) / TIME_BURN) * t + MASS_BEFORE
        Inertia = ((INERTIA_AFTER - INERTIA_BEFORE) / TIME_BURN) * t + INERTIA_BEFORE
        np.nan_to_num(Inertia, copy=False)

        thrust_data_number = math.floor(t / THRUST_TIME_SPAN)
        thrust = np.double(thrust_data_list[thrust_data_number])
        # thrust = np.double(thrust_data_list[thrust_data_number]) + (np.double(thrust_data_list[thrust_data_number + 1]) - np.double(thrust_data_list[thrust_data_number])) * (t - THRUST_TIME_SPAN * thrust_data_number) / THRUST_TIME_SPAN

        position_CG_end = ((POSITION_CG_END_AFTER - POSITION_CG_END_BEFORE) / TIME_BURN) * t + POSITION_CG_END_BEFORE
    else:  # 燃焼していない時の演算
        mass = MASS_AFTER
        Inertia = INERTIA_AFTER
        thrust = 0.0
        position_CG_end = POSITION_CG_END_AFTER

    # 座標変換行列もしくは基底変換行列(方向余弦行列)；GND系座標成分に作用させるとBODY系座標成分表示となる；BODY基底に作用させるとGND基底となる；逆行列は転置を取れば良い
    trans_mat = np.array([[Qtn[0, 0] ** 2 - Qtn[1, 0] ** 2 - Qtn[2, 0] ** 2 + Qtn[3, 0] ** 2, 2 * (Qtn[0, 0]*Qtn[1, 0] + Qtn[2, 0]*Qtn[3, 0]), 2 * (Qtn[0, 0]*Qtn[2, 0] - Qtn[1, 0]*Qtn[3, 0])],
                          [2 * (Qtn[0, 0]*Qtn[1, 0] - Qtn[2, 0]*Qtn[3, 0]), -1 * Qtn[0, 0] ** 2 + Qtn[1, 0] **
                           2 - Qtn[2, 0] ** 2 + Qtn[3, 0] ** 2, 2 * (Qtn[1, 0]*Qtn[2, 0] + Qtn[0, 0]*Qtn[3, 0])],
                          [2 * (Qtn[0, 0]*Qtn[2, 0] + Qtn[1, 0]*Qtn[3, 0]), 2 * (Qtn[1, 0]*Qtn[2, 0] - Qtn[0, 0]*Qtn[3, 0]), -1 * Qtn[0, 0] ** 2 - Qtn[1, 0] ** 2 + Qtn[2, 0] ** 2 + Qtn[3, 0] ** 2]], dtype=np.float32)
    np.nan_to_num(trans_mat, copy=False)

    velocity_wind_GND = wind_calc(CG_G[0, 0])
    np.nan_to_num(velocity_wind_GND, copy=False)
    velocity_airflow_BODY = np.dot(trans_mat, (velocity_wind_GND - CGv_G))
    np.nan_to_num(velocity_airflow_BODY, copy=False)

    # 動圧[Pa]の計算
    dpS = 0.5 * air_density_calc(CG_G[0]) * (np.linalg.norm(velocity_airflow_BODY) ** 2) * ROCKET_AREA

    # 迎角の計算
    if np.abs(velocity_airflow_BODY[0, 0] / (np.linalg.norm(velocity_airflow_BODY) + 1e-8)) < 1.0:
        alpha = math.acos(np.abs(velocity_airflow_BODY[0, 0] / (np.linalg.norm(velocity_airflow_BODY) + 1e-8)))
    else:
        alpha = 0.0
    # if np.abs(velocity_airflow_BODY[0, 0] / np.linalg.norm(velocity_airflow_BODY)) < 1.0:
    #     alpha = math.acos(-1 * velocity_airflow_BODY[0, 0] / np.linalg.norm(velocity_airflow_BODY))
    # elif -1 * velocity_airflow_BODY[0, 0] / np.linalg.norm(velocity_airflow_BODY) >= 1.0:
    #     alpha = 0.0
    # elif -1 * velocity_airflow_BODY[0, 0] / np.linalg.norm(velocity_airflow_BODY) <= -1.0:
    #     alpha = np.pi

    # 揚力・法線力の計算
    if alpha < ALPHA_MAX_RAD:
        Cl = CL_ALPHA_RAD * alpha
        Cn = CN_ALPHA_RAD * alpha
    else:
        Cl = CL_ALPHA_RAD * ALPHA_MAX_RAD
        Cn = CN_ALPHA_RAD * ALPHA_MAX_RAD
    # 抗力・軸力係数の計算
    Cd = CD_0 + CD_ALPHA_RAD * (alpha ** 2)
    
    # 機体座標系から見た迎角最大時の圧力中心の座標[m]
    CP_position_alphamax_BODY = np.array([[-1 * (position_CG_end - POSITION_CP_ALPHAMAX_END)], [0.0], [0.0]], dtype=np.float32)
    # 機体座標系から見た迎角０の時の圧力中心の位置[m]
    CP_position_alpha0_BODY = np.array([[-1 * (position_CG_end - POSITION_CP_ALPHA0_END)], [0.0], [0.0]], dtype=np.float32)
    # 機体座標系から見た現在の迎角における圧力中心の位置[m]
    if alpha < ALPHA_MAX_RAD:
        CP_position_BODY = CP_position_alphamax_BODY + (CP_position_alpha0_BODY - CP_position_alphamax_BODY) * cos((np.pi * alpha) / (2 * ALPHA_MAX_RAD))
    else:
        CP_position_BODY = CP_position_alphamax_BODY

    # 機体座標系から見た推力の作用点ベクトル[m]
    position_force_thrust_BODY = np.array([[-1 * position_CG_end], [0.0], [0.0]], dtype=np.float32)
    
    # Fstの算出
    Fst = - CP_position_BODY[0] / BODY_LENGTH

    if (AERODYNAMICS_STATUS == 0):
        force_drag_BODY = dpS * Cd * velocity_airflow_BODY / \
            (np.linalg.norm(velocity_airflow_BODY) + 1e-8)
        force_lift_BODY = dpS * Cl * np.cross(np.cross(velocity_airflow_BODY.T, np.array([1.0, 0.0, 0.0])), velocity_airflow_BODY.T).T / (
            np.linalg.norm(np.cross(np.cross(velocity_airflow_BODY.T, np.array([1.0, 0.0, 0.0])), velocity_airflow_BODY.T)) + 1e-8)
    elif (AERODYNAMICS_STATUS == 1):
        force_drag_BODY = dpS * CD_0 * \
            np.array([[velocity_airflow_BODY[0, 0] /
                     abs(velocity_airflow_BODY[0, 0] + 1e-8)], [0.0], [0.0]])
        force_lift_BODY = dpS * Cn * np.array([[0.0], [velocity_airflow_BODY[1, 0]], [velocity_airflow_BODY[2, 0]]]) / (
            np.linalg.norm(np.array([[0.0], [velocity_airflow_BODY[1, 0]], [velocity_airflow_BODY[2, 0]]])) + 1e-8)

    torque_drag_BODY = np.cross(CP_position_BODY.T, force_drag_BODY.T).T
    torque_lift_BODY = np.cross(CP_position_BODY.T, force_lift_BODY.T).T

    force_gravity_GND = np.array([[-1 * mass * G], [0.0], [0.0]])
    force_thrust_BODY = np.array([[thrust], [0.0], [0.0]])
    torque_thrust_BODY = np.cross(
        position_force_thrust_BODY.T, force_thrust_BODY.T).T

    if (ATTENUATION_STATUS == 1):
        torque_attenuation_BODY = 0.5 * dpS * np.array([[ROLL_DUMP * BODY_DIAMETER * angle_velocity[0, 0]], [PITCH_DUMP * BODY_LENGTH * angle_velocity[1, 0]], [PITCH_DUMP * BODY_LENGTH * angle_velocity[2, 0]]]) / (np.linalg.norm(velocity_airflow_BODY) + 1e-8)

    if sts == 1:
        torque_reaction_BODY = - (torque_lift_BODY + torque_drag_BODY + torque_thrust_BODY + torque_attenuation_BODY)

        if (force_thrust_BODY + force_lift_BODY + force_drag_BODY + np.dot(trans_mat, force_gravity_GND))[0, 0] < 0.0:
            force_reaction_BODY = -(force_thrust_BODY + force_lift_BODY + force_drag_BODY + np.dot(trans_mat, force_gravity_GND))
        else:
            force_reaction_BODY = np.array([[0.0], [-(force_thrust_BODY[1, 0] + force_lift_BODY[1, 0] + force_drag_BODY[1, 0] + np.dot(trans_mat, force_gravity_GND)[1, 0])], [-(force_thrust_BODY[2, 0] + force_lift_BODY[2, 0] + force_drag_BODY[2, 0] + np.dot(trans_mat, force_gravity_GND)[2, 0])]])

    if sts == 3:
        #force_drag_BODY = dpS / ROCKET_AREA * cds_para_now * velocity_airflow_BODY / np.linalg.norm(velocity_airflow_BODY)
        force_drag_BODY = np.zeros((3, 1), dtype=np.float32)  # デバッグ用
        force_lift_BODY = np.zeros((3, 1), dtype=np.float32)
        torque_drag_BODY = np.zeros((3, 1), dtype=np.float32)
        torque_lift_BODY = np.zeros((3, 1), dtype=np.float32)
        torque_attenuation_BODY = np.zeros((3, 1), dtype=np.float32)

    accel_GND = (force_gravity_GND + np.dot(trans_mat.T, (force_thrust_BODY +
                 force_lift_BODY + force_drag_BODY + force_reaction_BODY))) / mass

    if sts == 3:
        accel_GND = np.zeros((3, 1), dtype=np.float32)
        CGv_G = np.array([[-v_para_now], [velocity_wind_GND[1, 0]],[velocity_wind_GND[2, 0]]], dtype = np.float32)

    angle_accel_BODY = np.dot(np.linalg.inv(Inertia), torque_thrust_BODY + torque_drag_BODY + torque_lift_BODY +
                              torque_attenuation_BODY + torque_reaction_BODY - np.cross(angle_velocity.T, np.dot(trans_mat, angle_velocity).T).T)

    np.nan_to_num(accel_GND, copy=False)
    np.nan_to_num(angle_accel_BODY, copy=False)

    Qtn_time_develop_coefficient = np.array([[0.0,                      angle_velocity[2, 0],       -1 * angle_velocity[1, 0],  angle_velocity[0, 0]],
                                             [-1 * angle_velocity[2, 0], 0.0,
                                            angle_velocity[0, 0],       angle_velocity[1, 0]],
                                             [angle_velocity[1, 0],      -1 * angle_velocity[0, 0],
                                              0.0,                        angle_velocity[2, 0]],
                                             [-1 * angle_velocity[0, 0], -1 * angle_velocity[1, 0],  -1 * angle_velocity[2, 0],  0.0]], dtype=np.float32)
    np.nan_to_num(Qtn_time_develop_coefficient, copy=False)

    # 行列の掛け算ってサイズどうなるの
    Qtn_developer = np.dot(Qtn_time_develop_coefficient, Qtn)

    # if check == 1:
    #     w.writerow([t,CG_G[0],CG_G[1],CG_G[2],np.linalg.norm(CGv_G),np.linalg.norm(velocity_airflow_BODY),180*alpha/np.pi,force_reaction_BODY[0,0],thrust,])

    return np.array([CGv_G[0],
                    CGv_G[1],
                    CGv_G[2],
                    accel_GND[0],
                    accel_GND[1],
                    accel_GND[2],
                    angle_accel_BODY[0],
                    angle_accel_BODY[1],
                    angle_accel_BODY[2],
                    Qtn_developer[0],
                    Qtn_developer[1],
                    Qtn_developer[2],
                    Qtn_developer[3],
                    t,
                    X_arg[0],
                    X_arg[1],
                    X_arg[2],
                    X_arg[3],
                    X_arg[4],
                    X_arg[5],
                    X_arg[6],
                    X_arg[7],
                    X_arg[8],
                    X_arg[9],
                    X_arg[10],
                    X_arg[11],
                    X_arg[12],
                    np.linalg.norm(X_arg[9:13]),
                    np.linalg.det(trans_mat),
                    mass,
                    thrust,
                    position_CG_end,
                    np.dot(trans_mat, X_arg[3:6])[0],
                    np.dot(trans_mat, X_arg[3:6])[1],
                    np.dot(trans_mat, X_arg[3:6])[2],
                    np.linalg.norm(np.dot(trans_mat, X_arg[3:6])),
                    np.dot(trans_mat, velocity_wind_GND)[0],
                    np.dot(trans_mat, velocity_wind_GND)[1],
                    np.dot(trans_mat, velocity_wind_GND)[2],
                    np.linalg.norm(np.dot(trans_mat, velocity_wind_GND)),
                    velocity_airflow_BODY[0, 0],
                    velocity_airflow_BODY[1, 0],
                    velocity_airflow_BODY[2, 0],
                    np.linalg.norm(velocity_airflow_BODY),
                    air_density_calc(CG_G[0]),
                    dpS / ROCKET_AREA,
                    180 * alpha / np.pi,
                    Cl,
                    Cn,
                    Cd,
                    -CP_position_BODY[0, 0],
                    Fst,
                    force_lift_BODY[0, 0],
                    force_lift_BODY[1, 0],
                    force_lift_BODY[2, 0],
                    np.linalg.norm(force_lift_BODY),
                    force_drag_BODY[0, 0],
                    force_drag_BODY[1, 0],
                    force_drag_BODY[2, 0],
                    np.linalg.norm(force_drag_BODY),
                    torque_lift_BODY[0, 0],
                    torque_lift_BODY[1, 0],
                    torque_lift_BODY[2, 0],
                    np.linalg.norm(torque_lift_BODY),
                    torque_drag_BODY[0, 0],
                    torque_drag_BODY[1, 0],
                    torque_drag_BODY[2, 0],
                    np.linalg.norm(torque_drag_BODY),
                    force_reaction_BODY[0, 0],
                    force_reaction_BODY[1, 0],
                    force_reaction_BODY[2, 0],
                    torque_reaction_BODY[0, 0],
                    torque_reaction_BODY[1, 0],
                    torque_reaction_BODY[2, 0],
                    np.dot(trans_mat, accel_GND)[0],
                    np.dot(trans_mat, accel_GND)[1],
                    np.dot(trans_mat, accel_GND)[2],
                    np.sqrt(X_arg[1] ** 2 + X_arg[2] ** 2),
                    sts,
                    cds_para_now,
                    torque_attenuation_BODY[0, 0],
                    torque_attenuation_BODY[1, 0],
                    torque_attenuation_BODY[2, 0],
                    v_para_now,
                    velocity_wind_GND[0, 0],
                    velocity_wind_GND[1, 0],
                    velocity_wind_GND[2, 0],
                     ], dtype=np.float32)


def finalization():
    print(landing_result)

    with open('./result2_check/' + TIME_NOW + '/profile.csv', 'w', encoding='utf_8_sig') as file2:

        file2.write("風速の数,")
        file2.write(str(WIND_NUM) + "\n")
        file2.write("方位の数,")
        file2.write(str(AZIMUTH_NUM) + "\n")
        file2.write("地図の北に対するランチャ方位角,")
        file2.write(str(LAUNCHER_AZUMITH_DEG) + "\n")
        file2.write("ランチャ仰角,")
        file2.write(str(LAUNCH_ANGLE_DEG) + "\n")
        file2.write("弾道or開傘orタイマ,")
        file2.write(str(RECOVERY_STATUS) + "\n")

        for i in range(AZIMUTH_NUM):
            for j in range(WIND_NUM):

                landing_point_x_NORTH = landing_result[j, i, 0, 0] * cos(
                    - np.pi * LAUNCHER_AZUMITH_DEG / 180.0) - landing_result[j, i, 0, 1] * sin(- np.pi * LAUNCHER_AZUMITH_DEG / 180.0)
                landing_point_y_NORTH = landing_result[j, i, 0, 0] * sin(
                    - np.pi * LAUNCHER_AZUMITH_DEG / 180.0) + landing_result[j, i, 0, 1] * cos(- np.pi * LAUNCHER_AZUMITH_DEG / 180.0)

                file2.write(str(landing_point_x_NORTH) + "," +
                            str(landing_point_y_NORTH) + ",")
            file2.write("\n")

    with open('./result2_check/' + TIME_NOW + '/' + str(int(LAUNCH_ANGLE_DEG)) + 'deg_' + str(RECOVERY_STATUS) + '.csv', 'w', encoding='utf_8_sig') as file2:

        file2.write("緯度,経度" + "\n")

        for i in range(AZIMUTH_NUM):
            for j in range(WIND_NUM):

                output_longitude = LAUNCHER_LONGITUDE + (landing_result[j, i, 0, 0] * sin(- np.pi * LAUNCHER_AZUMITH_DEG / 180.0) +
                                                         landing_result[j, i, 0, 1] * cos(- np.pi * LAUNCHER_AZUMITH_DEG / 180.0)) * 180 / np.pi / 6356752.314
                output_latitude = LAUNCHER_LATITUDE + (landing_result[j, i, 0, 0] * cos(- np.pi * LAUNCHER_AZUMITH_DEG / 180.0) - landing_result[j, i, 0, 1] * sin(
                    - np.pi * LAUNCHER_AZUMITH_DEG / 180.0)) * 180 / np.pi / 6378137 / cos(np.pi * LAUNCHER_LONGITUDE / 180.0)

                file2.write(str(output_longitude) + "," +
                            str(output_latitude) + ",\n")

    # import os
    # os.system('play -n synth %s sin %s' % (30, 800))
    print('終了')


# 全ての処理を開始するおまじない
if __name__ == "__main__":
    main()

#                                       .(,
#                                    .,"^ 4,
#                                  .#^`... W.
#                               .," .....`.(b
#                             .d=....`..`...(b
#                          `.J^...`.`.``.`...d,
#                          .3 ....````.`.....-N
#                        .#~....``.``````.`..`,b
#                       .D ...``..`.`.``..`....W-
#                      .F....`.`.```.``.`......(b
#                     .F.`..`..````.``.``..`.`..H.
#                    .P `.`````.`.````.`..`..`..-[
#                    d``..`.............`...`...-F
#                   .%....+!,""""T+,.``..`.`..`..F
#                  .@,"~..`.......... 4i,`..`....F
#                  (=  ........~~..~....(5, `..`(\
#                .@  .......~.~----.~.~..._"a.`.d`
#               .d....   ...(+MN,  _"a-.~...-4,.F                          `./#
#          .,7"^  `.?O. `.._PdH#9"5J  ?e..~...J#                          .Y`.'
#      .,Tl............U. .`_?``..._Tj,3.~.~.~_W.                       .=  .t
#      ?+,`....`......(J^``.``..`.`.........~..(b                    .,=  `.F
#         _"OJ.-..-(,Y'  .- .``.`.`.`...~.~...~.N                  .J^ .`..$    ..J>
#              M~ ......~_...`.`.`.......~.~.~..d`               .J^..-`..b.Z"7! J^
#              # ...~.~..``.`.`.  -..(-v7""""""""""""""TGu(..  .J^......(=  `.`.J`
#            . ' .~..~...._.......(7!.......... .-.-......   ?"5a.-.~._J! .....D
#          .Y ...~..~.........~..................~.~.~...~.....   ?" .P ....(d>......
#         (^...~............................~....._..._..............t...~(J3._ _ .Y`
#        J` .~.`   `  .....`..`..................`...`..``..`.`....(^...(J5.....J"
#       .^  -.(++&+&,.     `.`.`..............`..`.`..`.`.``.`.`... .~(d3..._(T^
#      .F .dmmmmmmqqqqN-.    .`.`......`.```.`..`.`.`..`.`..`.``.`..(#!..~(JM+(....
#      (!.ZXWHmqqmmmqqqqmH,    ...`..`.`..`.`.`..`..`.`..``.``.``.."_..(J9!.`...T=
#     .F.~??!?7???????7UqmH,   .`.``..`.`..`.`.`..`..`.``..`.`..   -(v"~.._(J9^
#     ,':..... .......-JUWmH    .`...`...`...`..`.`...........`..  ..~.(-T=`      ..jv7C<(..
#     .  .  t:  W]   -   ,qm!   .`..........................._......(JW?h_      .O>-(<<?11&..
#     F(    v   ,] ?jh; qHqq~  .`.................~................J=JdRd]     ,C______---.?7O&((x:
#     F.    `    b ..mL (qmH  .`.............~.~..~.~~.~~.~.~....(VjjMWNW#_  _(-.~...-.---~_<>1z7`
#     G..(.   (  . dmmK ,mm\  ..`.....~..~.~..~..~....~..~..~.~(J^edHMHMpN} _<<!~_.:~~:~<~((vC~....
#     ,]?tOzwdmmmHkmmHqWHm^  .`.`....~..~..~...~..~..~......(J5,(JNW#dSMfN\  __._-______<<<~~_~((v!
#      5 (HH....(l.-.-+mY`  `.`.(. _..~..~..~...~..~....((J=  -\dWMMSdQ#W#>  .----.--....  .(v7!  ++-
#      ,,  ?HqqqmqqqqqY`  `.``.` .T&.._``_...~..._((.J"=` .~.(@dNMW#d#MWM:    -....    ....      ._J>
#         (.. ~_?777  ``.``..`.` .....~7""71i(?7<~.........(J8dNMHMW#dMM$  (-.  .._!<<:-......(++!(C
#         .N,........`..`..`..`......~.......-......(,.-(J#=+MNM0dNMHNM^  -i_x<-.....  .-.__((+<((>
#           ?G,.........-...`.`....~..~.~.~.._?7THMHMMHHMWMNmdSQMHBQ#^     ?COo~.  `_(((((.++(+r7!
#             .4J._.................~..~..~.((J"!_?HNNNNMMHC!_""HdB^          ?r&-<<<_(JZ^~?!!`
#                ."6J--.......~.~...~..((JY=_`         `7_'                     _77777!`
#                      ?7"""Tw;`<++x7"^`
