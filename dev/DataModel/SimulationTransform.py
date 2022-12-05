import pandas as pd 
import math 
import pymap3d as pm

class SimulationTransform:
    def __init__(self, offsets):
        self.gps_data = pd.DataFrame()
        self.attitude_data = pd.DataFrame()
        self._x_o = offsets[0]
        self._y_o = offsets[1]
        self._z_o = offsets[2]
        self._angle_offset = offsets[3]

    def deg_2_dec(self, coord, dir):
        dir = 1
        if dir == 'S' or dir == 'W': dir = -1
        deg = math.trunc(coord/100)
        dec = (coord/100 - deg)*(10/6)
        return dir*(deg + dec)

    def get_frame_dec(self):
        transformed_heading = self.attitude_data[['unix_time', 'head_deg']]
        transformed_locations = self.gps_data[
            ['unix_time', 'lat','lat_dir','lon','lon_dir', 'altitude']]

        transformed_data = pd.merge(
            transformed_heading,
            transformed_locations, 
            on='unix_time', how='outer')

        transformed_data = transformed_data.fillna(method='bfill').fillna(method='ffill')
            
        #pd.concat([transformed_locations, transformed_heading], axis = 1)  
                #lat: ddmm.mm dir  
        transformed_data['lat'] =  transformed_data.apply(
            lambda x: self.deg_2_dec(x.lat, x.lat_dir), 
            axis = 1
            )

        #lon: ddmm.mm dir 
        transformed_data['lon'] = transformed_data.apply(
            lambda x: self.deg_2_dec(x.lon, x.lon_dir),
            axis = 1
            )
        transformed_data = transformed_data.dropna().sort_values(by=['unix_time'])  
        return transformed_data

    def adjust_2_sim(self, num, d_len = 15):
        nstr = str(num)
        if nstr.find('.') is not -1: d_len+=1
        if nstr.find('-') is not -1: d_len+=1 
        if len(nstr) < d_len and nstr.find('.') is not -1:
            nstr = nstr.ljust(d_len-1,'0')
            nstr = nstr.ljust(d_len,'1')
        if len(nstr) < d_len and nstr.find('.') is -1:
            nstr = (nstr + '.').ljust(d_len-2,'0')
            nstr = nstr.ljust(d_len-1,'1')
        if len(nstr) > d_len: nstr = nstr[:d_len]

        return nstr

    def get_converted_frame(self): 
        temp = self.get_frame_dec()

        temp['unix_time'] = temp.apply(
            lambda x: self.adjust_2_sim(x.unix_time, 11),
            axis=1
        ) 

        temp['V1x'] = temp.apply(
            lambda x: self.adjust_2_sim(pm.geodetic2enu(x.lat, x.lon, x.altitude, self._y_o, self._x_o, self._z_o)[0]),
            axis=1
        ) 

        temp['V1y'] = temp.apply(
            lambda x: self.adjust_2_sim(pm.geodetic2enu(x.lat, x.lon, x.altitude, self._y_o, self._x_o, self._z_o)[1]),
            axis=1
        ) 

        temp['head_deg'] = temp.apply(
            lambda x: self.adjust_2_sim(x.head_deg),
            axis=1
        )

        unity_frame = pd.concat([
            temp['unix_time'].astype('string'),
            temp['V1x'].astype('string'), 
            temp['V1y'].astype('string'), 
            temp['head_deg'].astype('string')],
            axis = 1)

        unity_frame = unity_frame.rename(columns={
                "unix_time": "TimeInSecondsPosix", 
                "head_deg": "V1Heading"
                }
            )

        return unity_frame