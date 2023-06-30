import pandas as pd 
import math 
import pymap3d as pm
from utils.Point import Point

class SimulationTransform:
    def __init__(self, offsets=[0,0,0,0,0], join_type = 'merge'):
        self.gps_data = pd.DataFrame()
        self.attitude_data = pd.DataFrame()
        self._x_o = offsets[0]
        self._y_o = offsets[1]
        self._z_o = offsets[2]
        self._heading_offset = offsets[3]
        self._heading_dir = offsets[4]
        self._join_type = join_type
        self.nm_in_deg = 60
        self.m_in_nm = 1852
        self.mps_in_kn = 0.51444

    def deg_2_dec(self, coord, dir):
        dir = 1
        if dir == 'S' or dir == 'W': dir = -1
        deg = math.trunc(coord/100)
        dec = (coord/100 - deg)*(10/6)
        return dir*(deg + dec)
    
    def dec_2_deg(self, coord, direction='lon'):
        dir = ''
        deg = 0
        
        if (direction == 'lon'):
            if (coord < 0): dir = 'W'
            else: dir = 'E'
        else:
            if (coord < 0): dir = 'S'
            else: dir = 'N'

        deg = int(coord) 
        deg = deg*100 + (coord - deg)*60
        return deg, dir


    def get_frame_dec(self):
        transformed_heading = self.attitude_data[['unix_time', 'head_deg', 'roll_deg', 'pitch_deg']]
        transformed_locations = self.gps_data[
            ['unix_time', 'lat','lat_dir','lon','lon_dir', 'altitude']]

        if self._join_type == "merge":
            transformed_data = pd.merge(
                transformed_heading,
                transformed_locations, 
                on='unix_time', how='outer')
        else:
            transformed_heading = transformed_heading[['head_deg', 'roll_deg', 'pitch_deg']]
            transformed_data =  pd.concat([transformed_locations, transformed_heading], axis = 1)

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
    
    def get_xyz(self, northings, eastings, altitude):
        x,y,_ = pm.geodetic2enu(northings, eastings, altitude, self._y_o, self._x_o, self._z_o) 
        z = altitude - self._z_o
        return x,y,z
    
    def coords_to_xyz(self, northings, eastings, altitude, y_o, x_o, z_o):
        x,y,_ = pm.geodetic2enu(northings, eastings, altitude, y_o, x_o, z_o) 
        z = altitude - z_o
        return x,y,z

    def xyz_to_coords(self, x, y, lat_o, lon_o, h_o = 0, z = 0):
        lat, lon, _ = pm.enu2geodetic(x, y, z, lat_o, lon_o, h_o)
        return lat, lon
    
    def kn_to_nms(self, kn):
        nms = kn/3600
        return nms

    def nm_to_deg(self, nm):
        deg = nm / self.nm_in_deg
        return deg
    
    def m_to_nm(self, m):
        nm = m / self.m_in_nm
        return nm

    def mps_to_kn(self, mps):
        kn = mps * 1.94384449
        return kn
    
    def kn_to_mps(self, knot):
        return knot * self.mps_in_kn
    
    # Given three collinear points p, q, r, the function checks if 
    # point q lies on line segment 'pr' 
    def onSegment(self, p, q, r):
        if ( (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and 
                (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))):
            return True
        return False

    def orientation(self, p, q, r):
        # to find the orientation of an ordered triplet (p,q,r)
        # function returns the following values:
        # 0 : Collinear points
        # 1 : Clockwise points
        # 2 : Counterclockwise
        # See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/ 
        # for details of below formula. 
        val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y))
        if (val > 0):

            # Clockwise orientation
            return 1
        elif (val < 0):

            # Counterclockwise orientation
            return 2
        else:

            # Collinear orientation
            return 0

    # The main function that returns true if 
    # the line segment 'p1q1' and 'p2q2' intersect.
    def doIntersect(self,p1,q1,p2,q2):

        # Find the 4 orientations required for 
        # the general and special cases
        o1 = self.orientation(p1, q1, p2)
        o2 = self.orientation(p1, q1, q2)
        o3 = self.orientation(p2, q2, p1)
        o4 = self.orientation(p2, q2, q1)

        # General case
        if ((o1 != o2) and (o3 != o4)):
            return True

        # Special Cases

        # p1 , q1 and p2 are collinear and p2 lies on segment p1q1
        if ((o1 == 0) and self.onSegment(p1, p2, q1)):
            return True

        # p1 , q1 and q2 are collinear and q2 lies on segment p1q1
        if ((o2 == 0) and self.onSegment(p1, q2, q1)):
            return True

        # p2 , q2 and p1 are collinear and p1 lies on segment p2q2
        if ((o3 == 0) and self.onSegment(p2, p1, q2)):
            return True

        # p2 , q2 and q1 are collinear and q1 lies on segment p2q2
        if ((o4 == 0) and self.onSegment(p2, q1, q2)):
            return True

        # If none of the cases
        return False
        
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

        temp['altitude'] = temp.apply(
            lambda x: self.adjust_2_sim(x.altitude - self._z_o),
            axis=1
        ) 

        temp['head_deg'] = temp.apply(
            lambda x: self.adjust_2_sim((self._heading_dir*x.head_deg) - self._heading_offset),
            axis=1
        )

        temp['roll_deg'] = temp.apply(
            lambda x: self.adjust_2_sim(x.roll_deg),
            axis=1
        )

        temp['pitch_deg'] = temp.apply(
            lambda x: self.adjust_2_sim(x.pitch_deg),
            axis=1
        )

        unity_frame = pd.concat([
            temp['unix_time'].astype('string'),
            temp['V1x'].astype('string'), 
            temp['V1y'].astype('string'), 
            temp['head_deg'].astype('string'),
            temp['pitch_deg'].astype('string'),
            temp['roll_deg'].astype('string'),
            temp['altitude'].astype('string')],
            axis = 1)

        unity_frame = unity_frame.rename(columns={
                "unix_time": "TimeInSecondsPosix", 
                "head_deg": "V1Heading"
                }
            )

        return unity_frame