#renew_dropplot_fullplow.pyのデータ処理用の関数が収納されています。
#以下の関数、process_data()のパスをいい感じにいじってください。初期状態ではほりつかさ用になってます。


NUMBER_OF_WIND = 8                                                                  #風の数
NUMBER_OF_ANGLE = 8                                                                 #風の方位の数
ANGLE = [0, 45, 90, 135, 180, 225, 270, 315]                                        #ロケットの方位角。シミュレーターを参照
LON0, LAT0 = 140.012103, 40.249764                                                  #射点の(経度、緯度)
DIR_NAME = "2023-0617-232416"                                                       #CSVを探索するディレクトリ
PATH = "C:\\Users\\bestt\\Documents\\NRD_aerodynamics_3Dsimulation\\result\\"       #path。VScodeではダブルスラッシュに変更してください。


import pyproj
import numpy as np
import pandas as pd
from math import log, tan, pi, cos
import math


#シミュレータのＣＳＶからｚ、ｙ平面のデータを作成する。ｚが横、ｙは縦。慣習上、x, yに対応させる。
def process_data():#シミュレータ内部のresultフォルダの内、処理してほしいフォルダの名前を入れる。例、2023-0604-084950 半角
    y, z = [],[]

    #ｗind2の時の８方位との落下地点を得る。y,z平面上のデータ
    for j in range(NUMBER_OF_WIND):
    
        y_temp, z_temp = [], []

        for i in range(NUMBER_OF_ANGLE):
        
            filename = PATH + DIR_NAME +"\\history_" + str(ANGLE[i]) + "deg_wind" + str(j) + ".csv"
            
        #ndarryに変更
            data_1 = pd.read_csv(filename).values
    
        #落下地点
            y_temp.append(data_1[-1][2])    #縦 yとしたい
            z_temp.append(data_1[-1][3])    #横 xとしたい

        y_temp.append(y_temp[0])
        z_temp.append(z_temp[0])

        y.append(y_temp)
        z.append(z_temp) 

    return z, y #横、縦で返す


#x-y平面で原点からの方位角と距離を計算する。この時、関数内部でy軸正の向きし縄稚シミュレーターでいう90degを北と定義している。更にpyprojのために時計周りを正としている。
#x-y座標を回転させたものを代入した時の場合を考えていない。
def calculate_azimuth_dist(x, y):
    azimuth = []
    dist = []
    v_north = [0,1]
    for i in range(8):
        x_y= np.array(list(zip(x[i], y[i])))
        azimuth_temp = []
        x_y_dist = []
        for j in range(9):
            x_y_dist.append(math.hypot(x_y[j, 0], x_y[j][1]))
            if x_y[j, 0] < 0:
                azimuth_temp.append(360 - (np.rad2deg(math.acos(np.dot(v_north, x_y[j]) / x_y_dist[j]))))#acosは[-pi/2, pi/2]
            else:
                azimuth_temp.append(np.rad2deg(math.acos(np.dot(v_north, x_y[j]) / x_y_dist[j])))#acosは[-pi/2, pi/2]
            
        azimuth.append(azimuth_temp)
        dist.append(x_y_dist)
        
    azimuth = np.array(azimuth)
    return azimuth, dist


#平面上のデータの原点の緯度経度を指定することにより、各点の緯度経度を得る
#平面の座標X, yと原点の緯度経度を渡す
def process_data_to_lonlat(x, y):
    
    #並列計算のためにndarryに変更
    x, y  = np.array(x), np.array(y)    #風の数 x (方角の数+1)  +1なのはプロットするときの都合上で、初めの値と終わりの値が同じ。
    lat0, lon0 = np.full(NUMBER_OF_ANGLE + 1, LAT0), np.full(NUMBER_OF_ANGLE + 1, LON0)
    
    #距離と方位角
    azimuth , dist = calculate_azimuth_dist(x, y)
    
    lon1, lat1 = [], []
    
    for i in range( NUMBER_OF_WIND ):
        print(f"出発点：{LAT0}N, {LON0}E")
        print(f"進む方向：{azimuth[i]}度、進む距離：{dist[i]}m")
        #計算
        grs80 = pyproj.Geod(ellps = "GRS80")
        lon1_temp, lat1_temp, back_azimuth = grs80.fwd(lon0, lat0, np.array(azimuth[i]), np.array(dist[i]))#ndarrayを渡す
        lon1.append(lon1_temp)
        lat1.append(lat1_temp)
        
    return np.array(lon1), np.array(lat1)


#経度座標から画像上の座標に変換。経度と地図を渡す
def lon_to_pixel(lon, map):
    from math import log, tan, pi, cos
    # 経度→タイル番号
    if not (-180 <= lon <= 180):
        lon = (lon + 180) % 360 - 180
    x = ((lon + 180.) / 360) * pow(2, map.zoom)
    # タイル番号→キャンバスの座標
    pixel = (x - map.x_center) * map.tile_size + map.width / 2
    return pixel


#緯度座標から画像上の座標に変換。緯度と地図を渡す
def lat_to_pixel(lat, map):
    # 緯度→タイル番号
    if not (-90 <= lat <= 90):
        lat = (lat + 90) % 180 - 90
    y = (1 - log(tan(lat * pi / 180) + 1 / cos(lat * pi / 180)) / pi) / 2 * pow(2, map.zoom)
    # タイル番号→キャンバスの座標
    pixel = (y - map.y_center) * map.tile_size + map.height / 2
    return int(pixel)


#緯度経度から画像上の座標に変換。経度と緯度と地図を渡す
# #mapにはstaticmapを用いて8oox800で作成してください。
#また、image=map.render(zoom=14, center=[140.012103,40.249764])で縮尺と中心座標を決定してください。
def lonlat_to_pixel(lon1, lat1, map):
    
    x_pix, y_pix = [], []
    
    for j in range(lon1.shape[0]):
        
        x_pix_temp, y_pix_temp= [], []

        for i in range(lon1.shape[1]):
        
            x_pix_temp.append(lon_to_pixel(lon1[j][i], map))
            y_pix_temp.append(lat_to_pixel(lat1[j][i], map)) 
            
        x_pix.append(x_pix_temp)
        
        y_pix.append(y_pix_temp)
        
    return x_pix, y_pix

