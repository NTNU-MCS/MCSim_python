import pandas as pd 
import math 

class SimulationTransform:
    def __init__(self, offsets):
        self.gps_data = pd.DataFrame()
        self.attitude_data = pd.DataFrame()
        self._x_offset = offsets[0]
        self._y_offset = offsets[1]
        self._angle_offset = offsets[2]

    def _add_offset(self, item, offset):
        return item + offset

    def deg_2_dec(self, coord, dir):
        dir = 1
        if dir == 'S' or dir == 'W': dir = -1
        deg = math.trunc(coord/100)
        dec = (coord/100 - deg)*(10/6)
        return dir*(deg + dec)

    def lonlat_2_en(self, long_lat):
        new_lon_lat = pd.DataFrame(columns=['lat', 'lon'])
        #lat: ddmm.mm dir  
        new_lon_lat['lat'] =  long_lat.apply(lambda x: self.deg_2_dec(x.lat, x.lat_dir), axis = 1)

        #lon: ddmm.mm dir 
        new_lon_lat['lon'] = long_lat.apply(lambda x: self.deg_2_dec(x.lon, x.lon_dir), axis = 1)
        return new_lon_lat

    def get_converted_frame(self):
        transformed_heading = self.attitude_data[['head_deg']]
        transformed_locations = self.gps_data[['lat','lat_dir','lon','lon_dir']]
        transformed_locations = self.lonlat_2_en(transformed_locations) 
        transformed_data = pd.concat([transformed_locations, transformed_heading], axis = 1)  
        transformed_data = transformed_data.dropna()

        transformed_data['lon'] = transformed_data.apply(
            lambda x: self._add_offset(x.lon, self._x_offset), 
            axis = 1
            )
        transformed_data['lat'] = transformed_data.apply(
            lambda x: self._add_offset(x.lat, self._y_offset), 
            axis = 1
            )
        transformed_data['head_deg'] = transformed_data.apply(
            lambda x: self._add_offset(x.head_deg, self._angle_offset)
            , axis = 1
            )

        return transformed_data
