import socket
import numpy as np 
import json
from simulation.SimulationTransform import SimulationTransform
from utils.DashboardWebsocket import DashboardWebsocket
from colav.ColavManager import ColavManager
import math


class SimulationServer:
    def __init__(self, buffer_size, data_logger, address= None,
                websocket = DashboardWebsocket, transform=SimulationTransform(), distance_filter=None, predicted_interval = 30 ,colav_manager = ColavManager):
        
        self._buffer = data_logger.sorted_data
        self._running = False 
        self.address = address
        self.transform = transform
        self.ais_history = dict()
        self.ais_history_len = 20
        self.distance_filter = distance_filter
        self.gunnerus_lat=None
        self.gunnerus_lon=None
        self._unity_filter = 0.1
        self.websocket = websocket
        self._colav_manager = colav_manager
        self._predicted_interval = predicted_interval
    
        if address is not None:
            self._ip = address[0]
            self._port = address[1]
            self.buffer_size = buffer_size
            self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _has_data(self, msg):
        msg_keys = msg.keys() 
        has_data = ("lat" in msg_keys and "lon" in msg_keys)
        return has_data
    
    def _is_moving(self, msg):
        msg_keys = msg.keys() 
        has_data = ("course" in msg_keys and "speed" in msg_keys)
        return has_data

    def stop(self):
        self._running = False
        print('Simulation Client stopped')

    def dist(self, p,q):
        return math.sqrt( (q[0] - p[0])**2 + (q[1] - p[1])**2 )

    def _validate_coords(self, msg, distance):
        if (self.gunnerus_lat is None or self.gunnerus_lon is None): 
            return True
        
        if (msg["message_id"].find("!AI") == 0): 
            if (self._has_data(msg)): 
                lat = float(msg["lat"])  
                lon = float(msg["lon"])
                
                p = [lat,lon]
                q = [self.gunnerus_lat,self.gunnerus_lon]

                return self.dist(p,q) < distance
            
        return True 
    
    def _set_predicted_position(self, msg):
        if self._is_moving(msg) and msg['speed'] > 0:
            speed = self.transform.kn_to_mps(msg['speed']) 
            x = math.sin(math.radians(msg['course'])) * speed * self._predicted_interval
            y = math.cos(math.radians(msg['course'])) * speed * self._predicted_interval
            lat_p, lon_p = self.transform.xyz_to_coords(x, y, float(msg["lat"]), float(msg["lon"]))
            msg['lat_p'] = lat_p
            msg['lon_p'] = lon_p

    def replace_outliers(self, outliers, array, diff_mean): 
        if np.size(outliers) == 0: return array

        for index in np.nditer(outliers):
            if index == np.size(array)-1:
                array[index] = array[index - 1] + diff_mean 
            else:
                array[index] = array[index + 1] - diff_mean
        return array

    def _filter_outliers(self, next = (0,0), hist=[], m=2):
        if len(hist) > 0.5*self.ais_history_len:
            print('1', hist)
            t_arr = np.array(hist)
            t1 = t_arr[:,0]
            t2 = t_arr[:,1]
            t1_diff = np.diff(t1)
            t2_diff = np.diff(t2)
            m_t1_diff = np.mean(t1_diff)
            m_t2_diff = np.mean(t2_diff)
            m1 =  np.mean(t1)
            m2 =  np.mean(t2)
            sdev1 = np.std(t1)
            sdev2 = np.std(t2)
            a = np.where((t1-m1) > 1.5 * sdev1)
            b = np.where((t2-m2) > 1.5 * sdev2)
            r_arr = self.replace_outliers(a, t1, m_t1_diff)
            r_arr2 = self.replace_outliers(b, t2, m_t2_diff)
            filtered = np.stack((r_arr, r_arr2)).T.tolist()
            print(filtered)
            return filtered
        else: return hist

    def _set_pos_history(self,msg):
        if (msg["message_id"].find("!AI") == 0):  

            if (self._has_data(msg)):
                message_id = msg["message_id"]
                lat = float(msg["lat"])  
                lon = float(msg["lon"])   

                if(message_id in self.ais_history.keys()):
                    self.ais_history[message_id]["pos_history"] = self._filter_outliers((lat, lon), self.ais_history[message_id]["pos_history"])     
                    self.ais_history[message_id]["pos_history"].append([lon,lat])

                    if(len(self.ais_history[message_id]["pos_history"]) > self.ais_history_len):
                        self.ais_history[message_id]["pos_history"].pop(0)
                else: 
                    self.ais_history[message_id] = dict()
                    self.ais_history[message_id]["pos_history"] = [[lon, lat]]

                msg['pos_history']  = self.ais_history[message_id]["pos_history"]

    def _compose_msg(self, msg, msg_type = 'datain'): 
        self._set_pos_history(msg)
        self._set_predicted_position(msg)
        return(json.dumps({
            "type": msg_type,
            "content": msg
            },default=str))
    
    def _compose_transformed_msg(self, msg):
        northings = self.transform.deg_2_dec(float(msg["lat"]), msg['lat_dir'])
        eastings = self.transform.deg_2_dec(float(msg["lon"]), msg['lon_dir'])
        altitude = msg['altitude'] 
        x,y,z = self.transform.get_xyz(northings, eastings, altitude)
        return(json.dumps({"message_id":"coords","x":x, "y":y, "z":z}, default=str))
    
    def _compose_ais_msg(self, msg): 
            northings = float(msg["lat"])
            eastings = float(msg["lon"])
            altitude = 0
            mmsi = msg['mmsi']
            heading = 0

            if ('heading' in msg.keys()):
                heading = msg['heading']

            x,y,z = self.transform.get_xyz(northings, eastings, altitude)
            return(json.dumps({"message_id": "ais","x":x, "y":y, "mmsi":mmsi, "heading": heading}, default=str)) 

    def _set_gunnerus_coords(self, msg): 
        self.gunnerus_lon = self.transform.deg_2_dec(float(msg["lon"]), msg['lon_dir'])
        self.gunnerus_lat = self.transform.deg_2_dec(float(msg["lat"]), msg['lat_dir'])

    def _send(self, message): 
        if self._validate_coords(message, self.distance_filter): 
            json_msg = self._compose_msg(message)

            if self.websocket.enable: 
                self.websocket.send(json_msg)

            valid_ais_msg = (
                message["message_id"].find("!AI") == 0 and
                message["message_id"].find("_ext_") != -1 and
                self._has_data(message)
                )
            
            if self.address is not None:

                if valid_ais_msg:  
                    self._colav_manager.update_ais_data(message)

                    if self._validate_coords(message, self._unity_filter):
                        msg = self._compose_ais_msg(message)
                        msg = msg.encode('ascii')
                        self.server.sendto(msg, (self._ip, self._port))

                if message["message_id"]=="$GPGGA_ext":  
                    self._set_gunnerus_coords(message)
                    msg = self._compose_transformed_msg(message)
                    msg = msg.encode('ascii')
                    self.server.sendto(msg, (self._ip, self._port))

                if message["message_id"]=="$GPRMC_ext": 
                    self._colav_manager.update_gunnerus_data(message)
                    pass

                if message["message_id"]=="$PSIMSNS_ext":
                    msg = json.dumps(message, default=str)
                    msg = msg.encode('ascii')
                    self.server.sendto(msg, (self._ip, self._port))
                
    def pop_buffer(self, index = None):
        if len(self._buffer) < 1: return

        if index is not None:
            return self._buffer.pop(index)
        else:
            return self._buffer.pop()
        
    def start(self):
        self._running = True 
        print('Simulation Client running...')

        while self._running:
            if len(self._buffer):
                self._send(self._buffer[0])
                self.pop_buffer(0)

        
