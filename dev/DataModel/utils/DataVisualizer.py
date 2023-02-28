import easygui
import pandas as pd 
import folium
import math 

path = easygui.fileopenbox()
df = pd.read_csv(path)
gunnerus_locations = df[['lat','lat_dir','lon','lon_dir']]

def lonlat_2_en(long_lat):
    new_lon_lat = pd.DataFrame(columns=['lat', 'lon'])
    #lat: ddmm.mm dir  
    new_lon_lat['lat'] =  long_lat.apply(lambda x: deg_2_dec(x.lat, x.lat_dir), axis = 1)

    #lon: ddmm.mm dir 
    new_lon_lat['lon'] = long_lat.apply(lambda x: deg_2_dec(x.lon, x.lon_dir), axis = 1)
    return new_lon_lat

def deg_2_dec(coord, dir):
    dir = 1
    if dir == 'S' or dir == 'W': dir = -1
    deg = math.trunc(coord/100)
    dec = (coord/100 - deg)*(10/6)
    return dir*(deg + dec)

gunnerus_locations = lonlat_2_en(gunnerus_locations)

map = folium.Map(location=[gunnerus_locations.lat.mean(), gunnerus_locations.lon.mean()], zoom_start=30, control_scale=True)

folium.PolyLine(gunnerus_locations, color='red', weight=3, opacity=0.8).add_to(map)
map.save('index.html')