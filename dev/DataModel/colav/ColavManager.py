from utils.DashboardWebsocket import DashboardWebsocket
from simulation.SimulationTransform import SimulationTransform
from colav.ARPA import ARPA
from colav.CBF import CBF
import json
import math
import copy
import time
import time  
from utils.Point import Point

class ColavManager:
    def __init__(self, enable = True, update_interval = 1, 
                safety_radius_m = 200, safety_radius_tol = 1.5, 
                max_d_2_cpa = 2000, gunnerus_mmsi ='' ,
                websocket=DashboardWebsocket,  dummy_gunnerus = None,
                dummy_vessel = None):
        
        self._cbf_message_id = 'cbf'
        self._arpa_message_id = 'arpa'
        self._gunnerus_data = {}
        self._ais_data = {}
        self.websocket = websocket
        self._running = False
        self.enable = enable
        self._update_interval = update_interval
        self.gunnerus_mmsi = gunnerus_mmsi
        self._timeout = time.time() + update_interval
        self._transform = SimulationTransform()
        self._prediction_interval = update_interval*2
        self._safety_radius_m = safety_radius_m
        self._safety_radius_tol = safety_radius_tol
        self._safety_radius_nm = self._transform.m_to_nm(safety_radius_m)
        self._safety_radius_deg = self._transform.nm_to_deg(self._safety_radius_nm)
        self._max_d_2_cpa = max_d_2_cpa
        self.dummy_gunnerus = dummy_gunnerus
        self.dummy_vessel = dummy_vessel

        self._arpa = ARPA(
            safety_radius_m= self._safety_radius_m,
            safety_radius_tol= self._safety_radius_tol, 
            max_d_2_cpa= self._max_d_2_cpa,
            transform= self._transform,
            gunnerus_mmsi=self.gunnerus_mmsi) 
        
        self._cbf = CBF(
            safety_radius_m= self._safety_radius_m,
            transform= self._transform)
    
    def update_gunnerus_data(self, data): 
        if self.dummy_gunnerus is not None:
            self._gunnerus_data = self.dummy_gunnerus
        else:
            self._gunnerus_data = data 
    
    def update_ais_data(self, data):
        if self.dummy_vessel is not None:
            message_id = "dummy"
            self._ais_data[message_id] = self.dummy_vessel

        message_id = data["message_id"]
        self._ais_data[message_id] = data  
    
    def _reset_timeout(self):
        self._timeout = time.time() + self._update_interval

    def stop(self):
        self._running = False
        self._cbf.stop()
        print('Colav Manager: Stop') 

    def _compose_colav_msg(self, msg, message_id):
        msg_type = 'datain'
            
        content = {
            'message_id': message_id,
            'data': msg
            }
        
        return(json.dumps({
            "type": msg_type,
            "content": content
            },default=str))

    def _update(self): 
        start = time.time()
        if self.dummy_vessel is not None:
            self.websocket.send(json.dumps({
            "type": 'datain',
            "content": self.dummy_vessel
            },default=str)
                )
        self._arpa.update_gunnerus_data(self._gunnerus_data)
        self._arpa.update_ais_data(self._ais_data)
        arpa_gunn_data, arpa_data = self._arpa.get_ARPA_parameters()  
        if (arpa_data and arpa_gunn_data): 
            converted_arpa_data = self._arpa.convert_arpa_params(arpa_data, arpa_gunn_data)
            self.websocket.send(self._compose_colav_msg(converted_arpa_data, self._arpa_message_id)) 
            self._cbf.update_cbf_data(arpa_gunn_data, arpa_data)
            cbf_data = self._cbf.get_cbf_data() 
            converted_cbf_data = self._cbf.convert_data(cbf_data)
            compose_cbf = self._compose_colav_msg(converted_cbf_data, self._cbf_message_id) 
            self.websocket.send(compose_cbf) 
        end = time.time()
        print("end update Arpa + CBF", end-start)

    def start(self):
        if self.enable: 
            self._running = True
            print('Colav Manager running...')

        while self._running:
            if time.time() > self._timeout and self._running:
                self._reset_timeout()
                self._update()
        



