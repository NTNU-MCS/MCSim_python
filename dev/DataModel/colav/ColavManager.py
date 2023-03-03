from utils.DashboardWebsocket import DashboardWebsocket
from simulation.SimulationTransform import SimulationTransform
import json
import math
import copy
import time  
from utils.Point import Point

class ColavManager:
    def __init__(self, enable = True, update_interval = 1, safety_radius_m = 200, 
                gunnerus_mmsi ='' ,websocket=DashboardWebsocket, dummy_gunnerus = None, dummy_vessel = None):
        self._gunnerus_data = {}
        self._ais_data = {}
        self.websocket = websocket
        self._running = False
        self.enable = enable
        self._update_interval = update_interval
        self.gunnerus_mmsi = gunnerus_mmsi
        self._timeout = time.time() + update_interval
        self._transform = SimulationTransform()
        self._prediction_interval = update_interval * 2
        self._safety_radius_m = safety_radius_m
        self._safety_radius_nm = self._transform.m_to_nm(safety_radius_m)
        self._safety_radius_deg = self._transform.nm_to_deg(self._safety_radius_nm)
        self.dummy_gunnerus =  dummy_gunnerus
        self.dummy_vessel = dummy_vessel
    
    def update_gunnerus_data(self, data):
        self._gunnerus_data = data 
    
    def update_ais_data(self, data):
        message_id = data["message_id"]
        self._ais_data[message_id] = data 
    
    def _reset_timeout(self):
        self._timeout = time.time() + self._update_interval

    def stop(self):
        self._running = False
        print('Colav Manager: Stop')

    def _create_segment(self, lon_o, lat_o, course, speed_kn, offset = 0): 
        speed_mps = self._transform.kn_to_mps(speed_kn)
        length = speed_mps * self._prediction_interval

        if (length < self._safety_radius_m): length = self._safety_radius_m

        dx = math.sin(math.radians(course))
        dy = math.cos(math.radians(course)) 

        offsetx = offset * math.sin(math.radians(course + 90))
        offsety = offset * math.cos(math.radians(course + 90))

        x = length * dx + offsetx
        y = length * dy + offsety
        lat,lon = self._transform.xyz_to_coords(x,y,lat_o, lon_o)  
        end = Point(lon,lat)

        x2 = -self._safety_radius_m * dx + offsetx
        y2 = -self._safety_radius_m * dy + offsety
        lat_oo, lon_oo = self._transform.xyz_to_coords(x2,y2,lat_o, lon_o)
        origin = Point(lon_oo, lat_oo)
        
        return origin, end
    
    def _create_static_segment(self, lon_o, lat_o, offset):
        length = self._safety_radius_m
        course = self._gunnerus_data['true_course'] + 90

        offsetx = offset * math.sin(math.radians(course + 90))
        offsety = offset * math.cos(math.radians(course + 90))

        dx = math.sin(math.radians(course))
        dy = math.cos(math.radians(course))
        x = length * dx + offsetx
        y = length * dy + offsety
        lat,lon = self._transform.xyz_to_coords(x,y,lat_o, lon_o)
        end = Point(lon,lat)

        x2 = -length * dx + offsetx
        y2 = -length * dy + offsety
        lat_oo, lon_oo = self._transform.xyz_to_coords(x2,y2,lat_o, lon_o)
        origin = Point(lon_oo, lat_oo)
        return origin, end
    
    def _get_gunnerus_segment(self, data, offset):
        gunn_lat = self._transform.deg_2_dec(
            data['lat'], 
            data['lat_dir']
            )
        
        gunn_lon = self._transform.deg_2_dec(
            data['lon'], 
            data['lon_dir']
            )
        
        gunn_course = data['true_course']
        gunn_speed = data['spd_over_grnd']
        
        gunn_o, gunn_t = self._create_segment(
            gunn_lon, 
            gunn_lat, 
            gunn_course, 
            gunn_speed,
            offset)
        
        return gunn_o, gunn_t
    
    def _has_coords(self, message): 
        msg_keys = message.keys() 
        has_data = ("lat" in msg_keys and "lon" in msg_keys)
        return has_data
    
    def _has_course(self, message): 
        msg_keys = message.keys() 
        has_data = ("course" in msg_keys and "speed" in msg_keys)
        return has_data
    
    def _get_ais_segment(self, ais_message, offset):
        ais_has_coords = self._has_coords(ais_message)

        if ais_has_coords:
            lon_o = ais_message['lon']
            lat_o = ais_message['lat']
            ais_has_course = self._has_course(ais_message)

            if ais_has_course and ais_message['speed'] > 0:
                course = ais_message['course']
                speed_kn = ais_message['speed']
                ais_o, ais_t = self._create_segment(
                    lon_o, 
                    lat_o, 
                    course, 
                    speed_kn,
                    offset)
            else:
                ais_o, ais_t = self._create_static_segment(lon_o, lat_o, offset)
                
            return ais_o, ais_t
        
        return None, None
    
    def _compose_intersect_msg(self, msg):
        msg_type = 'datain'
        
        content = {
            'message_id': "intersects",
            'intersects': msg
            }
        
        return(json.dumps({
            "type": msg_type,
            "content": content
            },default=str))
    
    def _check_intersects(self):
        msg = []
        gunnerus_or, gunnerus_tr = (None, None)
        gunnerus_ol, gunnerus_tl = (None, None)
        gunnerus_speed = None
        gunnerus_course = None

        if (self.dummy_gunnerus is not None):
            gunnerus_or, gunnerus_tr = self._get_gunnerus_segment(self.dummy_gunnerus, self._safety_radius_m)
            gunnerus_ol, gunnerus_tl = self._get_gunnerus_segment(self.dummy_gunnerus, -self._safety_radius_m)
            gunnerus_speed = self.dummy_gunnerus["spd_over_grnd"]
            gunnerus_course = self.dummy_gunnerus["true_course"]
        else:
            gunnerus_or, gunnerus_tr = self._get_gunnerus_segment(self._gunnerus_data, self._safety_radius_m)
            gunnerus_ol, gunnerus_tl = self._get_gunnerus_segment(self._gunnerus_data, -self._safety_radius_m)
            gunnerus_speed = self._gunnerus_data["spd_over_grnd"]
            gunnerus_course = self._gunnerus_data["true_course"]

        msg.append({
            "mmsi" : self.gunnerus_mmsi, 
            "originR":[gunnerus_or.x, gunnerus_or.y], 
            "targetR":[gunnerus_tr.x, gunnerus_tr.y],
            "originL":[gunnerus_ol.x, gunnerus_ol.y], 
            "targetL":[gunnerus_tl.x, gunnerus_tl.y],
            "course": gunnerus_speed,
            "speed": gunnerus_course
            }) 
        
        ais_data = copy.deepcopy(self._ais_data) 

        if (self.dummy_vessel is not None):
            ais_data['dummy'] =  self.dummy_vessel

        ais_msg_ids = ais_data.keys()  

        for ais_id in ais_msg_ids:
            if str(ais_data[ais_id]['mmsi']) == self.gunnerus_mmsi:
                continue
            ais_message = ais_data[ais_id]
            ais_or, ais_tr = self._get_ais_segment(ais_message, self._safety_radius_m)
            ais_ol, ais_tl = self._get_ais_segment(ais_message, -self._safety_radius_m)

            if (ais_or is not None):
                a = self._transform.doIntersect(gunnerus_ol, gunnerus_tl, ais_ol, ais_tl)
                b = self._transform.doIntersect(gunnerus_ol, gunnerus_tl, ais_or, ais_tr)
                c = self._transform.doIntersect(gunnerus_or, gunnerus_tr, ais_ol, ais_tl)
                d = self._transform.doIntersect(gunnerus_or, gunnerus_tr, ais_or, ais_tr)
                e = self._transform.doIntersect(gunnerus_tl, gunnerus_tr, ais_ol, ais_tl)
                f = self._transform.doIntersect(gunnerus_tl, gunnerus_tr, ais_or, ais_tr)
                intersects = a or b or c or d or e or f
                #if intersects:  
                if True:
                    msg.append({
                    "mmsi" : ais_data[ais_id]['mmsi'], 
                    "originL":[ais_ol.x, ais_ol.y], 
                    "targetL":[ais_tl.x, ais_tl.y],
                    "originR":[ais_or.x, ais_or.y], 
                    "targetR":[ais_tr.x, ais_tr.y]
                    })  
        return msg

    def _update(self): 
        msg = self._check_intersects()
        if(self.websocket.enable):
            self.websocket.send(self._compose_intersect_msg(msg)) 

    def start(self):
        if self.enable: 
            self._running = True
            print('Colav Manager running...')

        while self._running:
            if time.time() > self._timeout and self._running:
                self._reset_timeout()
                self._update()
        



