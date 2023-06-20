#審査書用のプロット用です。
#下の定数をいじってください。
#simudata2plotという自作モジュールがあるので読み込めるようにしておいてから実行してください。
#環境は.tomlを参考
import simudata2plot
from staticmap import StaticMap
from PIL import ImageDraw








#定数#############################################################################
NUMBER_OF_WIND = 8                                                                  #風の数
NUMBER_OF_ANGLE = 8                                                                 #風の方位の数
ANGLE = [0, 45, 90, 135, 180, 225, 270, 315]                                        #ロケットの方位角。シミュレーターを参照
LON0, LAT0 = 140.012103, 40.249764                                                  #射点の(経度、緯度)
DIR_NAME = "2023-0617-232416"                                                       #CSVを探索するディレクトリ
PATH = "C:\\Users\\bestt\\Documents\\NRD_aerodynamics_3Dsimulation\\result\\"       #path。デフォルトではほりつかさ仕様なので変更してください。フルパスで、result\\までお願いします.VScodeではダブルスラッシュに変更してください。
DATE = "2023-0617-232416"                                                           #シミュを動かしたときにのresultの下にあるフォルダ名にしてください。
ZOOMRATE = 17                                                                       #地図をどれだけズームするか
MAP_TILE = 'https://cyberjapandata.gsi.go.jp/xyz/std/{z}/{x}/{y}.png'
#MAP_TILE = 'https://cyberjapandata.gsi.go.jp/xyz/seamlessphoto/{z}/{x}/{y}.jpg'       #航空写真。ZOOMRATE16以上ならいける

##################################################################################














#z-y平面をシミュレーターのCSVから作成
x, y = simudata2plot.process_data()

#z-y平面の原点に緯度経度を与えて、データを緯度経度に変換
lon1, lat1 = simudata2plot.process_data_to_lonlat(x, y)

#緯度経度から地図画像上の座標に変換。地図を描画
map = StaticMap(1500, 1500, url_template=MAP_TILE)# 画像の横幅と縦幅と国土地理院の地理院タイルURLを指定

# 地図を描画した Pillow の PIL.Image オブジェクトを取得
# ズームレベルと経度・緯度のリストを指定(google mapで」しらべればわかる)
pic = map.render(zoom=ZOOMRATE, center=[140.012103,40.249764])

#プロット画像の座標を取得
x_pix, y_pix = simudata2plot.lonlat_to_pixel(lon1, lat1, map)

# Drawオブジェクト作成
draw = ImageDraw.Draw(pic)

#落下分散のプロット
for i in range(NUMBER_OF_WIND):
    x_y_pix = list(zip(x_pix[i][:], y_pix[i][:]))
    draw.polygon(x_y_pix, outline='black')

#射点の座標
x0_pix, y0_pix = simudata2plot.lon_to_pixel(LON0, map), simudata2plot.lat_to_pixel(LAT0, map)

#射点のプロット
draw.ellipse([(x0_pix-6, y0_pix-6), (x0_pix+6, y0_pix+6) ], fill = "red", outline='red', width = 1)

# 地図画像を保存
pic.save(f'dp_from_csv_{DATE}.png')
