import socket
import pandas as pd 
import json
from simulation.SimulationTransform import SimulationTransform
from utils.DashboardWebsocket import DashboardWebsocket
import math


class SimulationServer:
    def __init__(self, buffer_size, data_logger, address= None,
                websocket = DashboardWebsocket, transform=SimulationTransform(), distance_filter=None):
        
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
    
        if address is not None:
            self._ip = address[0]
            self._port = address[1]
            self.buffer_size = buffer_size
            self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _has_data(self, msg):
        msg_keys = msg.keys() 
        has_data = ("lat" in msg_keys and "lon" in msg_keys)
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
                lat = msg["lat"]  
                lon = msg["lon"]
                
                p = [lat,lon]
                q = [self.gunnerus_lat,self.gunnerus_lon]

                return self.dist(p,q) < distance
            
        return True 

    def _set_pos_history(self,msg):
        if (msg["message_id"].find("!AI") == 0): 

            if (self._has_data(msg)):
                message_id = msg["message_id"]
                lat = msg["lat"]  
                lon = msg["lon"]                     

                if(message_id in self.ais_history.keys()):
                    self.ais_history[message_id]["pos_history"].append([lon,lat])

                    if(len(self.ais_history[message_id]["pos_history"]) > self.ais_history_len):
                        self.ais_history[message_id]["pos_history"].pop(0)
                else: 
                    self.ais_history[message_id] = dict()
                    self.ais_history[message_id]["pos_history"] = [[lon, lat]]

                msg['pos_history']  = self.ais_history[message_id]["pos_history"]

    def _compose_msg(self, msg, msg_type = 'datain'): 
        self._set_pos_history(msg)
        return(json.dumps({
            "type": msg_type,
            "content": msg
            },default=str))
    
    def _compose_transformed_msg(self, msg):
        northings = self.transform.deg_2_dec(msg['lat'], msg['lat_dir'])
        eastings = self.transform.deg_2_dec(msg['lon'], msg['lon_dir'])
        altitude = msg['altitude'] 
        x,y,z = self.transform.get_xyz(northings, eastings, altitude)
        return(json.dumps({"message_id":"coords","x":x, "y":y, "z":z}, default=str))
    
    def _compose_ais_msg(self, msg): 
            northings = msg['lat']
            eastings = msg['lon']
            altitude = 0
            mmsi = msg['mmsi']
            heading = 0

            if ('heading' in msg.keys()):
                heading = msg['heading']

            x,y,z = self.transform.get_xyz(northings, eastings, altitude)
            return(json.dumps({"message_id": "ais","x":x, "y":y, "mmsi":mmsi, "heading": heading}, default=str)) 

    def _set_gunnerus_coords(self, msg): 
        self.gunnerus_lon = self.transform.deg_2_dec(msg['lon'], msg['lon_dir'])
        self.gunnerus_lat = self.transform.deg_2_dec(msg['lat'], msg['lat_dir'])

    def _send(self, message): 
        if self._validate_coords(message, self.distance_filter): 
            json_msg = self._compose_msg(message)

            if self.websocket.enable: 
                self.websocket.send(json_msg)
            if self.address is not None:
                if (message["message_id"].find("!AI") == 0 and 
                self._has_data(message) and 
                self._validate_coords(message, self._unity_filter)):
                    msg = self._compose_ais_msg(message)
                    msg = msg.encode('ascii')
                    self.server.sendto(msg, (self._ip, self._port))
                if message["message_id"]=="$GPGGA_ext": 
                    self._set_gunnerus_coords(message)
                    msg = self._compose_transformed_msg(message)
                    msg = msg.encode('ascii')
                    self.server.sendto(msg, (self._ip, self._port))
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

        
