from utils.DashboardWebsocket import DashboardWebsocket
from simulation.SimulationTransform import SimulationTransform
import json
import math
import copy
import time  
from utils.Point import Point

class ColavManager:
    def __init__(self, enable = True, update_interval = 1, safety_radius_m = 200, gunnerus_mmsi ='' ,websocket=DashboardWebsocket):
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

    def _create_segment(self, lon_o, lat_o, course, speed_kn): 
        speed_nms = self._transform.kn_to_nms(speed_kn)
        length = self._transform.nm_to_deg(speed_nms * self._prediction_interval)
        lon = length * math.sin(course) + lon_o
        lat = length * math.cos(course) + lat_o
        lon_oo = lon_o - self._safety_radius_deg * math.sin(course) 
        lat_oo = lat_o - self._safety_radius_deg * math.cos(course)
        origin =  Point(lon_oo, lat_oo)
        end = Point(lon,lat)
        return origin, end
    
    def _create_static_segment(self, lon_o, lat_o):
        length = self._safety_radius_deg
        course = self._gunnerus_data['true_course'] + 90
        lon = length * math.sin(course) + lon_o
        lat = length * math.cos(course) + lat_o
        lon_oo = lon_o - length * math.sin(course) 
        lat_oo = lat_o - length * math.cos(course) 
        origin =  Point(lon_oo, lat_oo) 
        end = Point(lon,lat)
        return origin, end
    
    def _get_gunnerus_segment(self):
        gunn_lat = self._transform.deg_2_dec(
            self._gunnerus_data['lat'], 
            self._gunnerus_data['lat_dir']
            )
        
        gunn_lon = self._transform.deg_2_dec(
            self._gunnerus_data['lon'], 
            self._gunnerus_data['lon_dir']
            )
        
        gunn_course = self._gunnerus_data['true_course']
        gunn_speed = self._gunnerus_data['spd_over_grnd']
        
        gunnerus_o, gunnerus_t = self._create_segment(
            gunn_lon, 
            gunn_lat, 
            gunn_course, 
            gunn_speed)
        
        return gunnerus_o, gunnerus_t
    
    def _has_coords(self, message): 
        msg_keys = message.keys() 
        has_data = ("lat" in msg_keys and "lon" in msg_keys)
        return has_data
    
    def _has_course(self, message): 
        msg_keys = message.keys() 
        has_data = ("course" in msg_keys and "speed" in msg_keys)
        return has_data
    
    def _get_ais_segment(self, ais_message):
        ais_has_coords = self._has_coords(ais_message)

        if ais_has_coords:
            lon_o = ais_message['lon']
            lat_o = ais_message['lat']
            ais_has_course = self._has_course(ais_message)

            if ais_has_course:
                course = ais_message['course']
                speed_kn = ais_message['speed']
                ais_o, ais_t = self._create_segment(
                    lon_o, 
                    lat_o, 
                    course, 
                    speed_kn)
            else:
                ais_o, ais_t = self._create_static_segment(lon_o, lat_o)
                
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
        gunnerus_o, gunnerus_t = self._get_gunnerus_segment()
        msg = []
        msg.append({
            "mmsi" : self.gunnerus_mmsi, 
            "origin":[gunnerus_o.x, gunnerus_o.y], 
            "target":[gunnerus_t.x, gunnerus_t.y],
            "course": self._gunnerus_data["true_course"],
            "speed": self._gunnerus_data["spd_over_grnd"]
            }) 
        ais_data = copy.deepcopy(self._ais_data)
        ais_msg_ids = ais_data.keys()  

        for ais_id in ais_msg_ids:
            if str(ais_data[ais_id]['mmsi']) == self.gunnerus_mmsi:
                continue
            ais_message = ais_data[ais_id]
            ais_o, ais_t = self._get_ais_segment(ais_message)
            if (ais_o is not None):
                intersects = self._transform.doIntersect(
                    gunnerus_o, 
                    gunnerus_t,
                    ais_o, 
                    ais_t)
                
                if intersects:  
                    print('\n\n',ais_data[ais_id]['mmsi'])
                    msg.append({
                    "mmsi" : ais_data[ais_id]['mmsi'], 
                    "origin":[ais_o.x, ais_o.y], 
                    "target":[ais_t.x, ais_t.y],
                    "course": ais_data[ais_id]["course"],
                    "speed": ais_data[ais_id]["speed"]
                    })  
        return msg

    def _update(self):
        msg = self._check_intersects()
        if(self.websocket.enable):
            self.websocket.send(self._compose_intersect_msg(msg))
        print('update', time.time(), msg)

    def start(self):
        if self.enable: 
            self._running = True
            print('Colav Manager running...')

        while self._running:
            if time.time() > self._timeout:
                self._reset_timeout()
                self._update()
        



