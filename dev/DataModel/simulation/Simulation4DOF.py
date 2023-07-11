import math 
import os
from pathlib import Path
import sys
from datetime import datetime 
import numpy as np
import json
from time import time

par_path = str(Path(os.path.dirname(__file__)).parents[2])
library_path = par_path + '\\lib'
sys.path.append(library_path)

module_path = par_path + '\\models\\RVG_maneuvering4DOF'
sys.path.append(module_path)

import kinematics as km
import visualization as viz
import Module_RVGManModel4DOF as model

from simulation.SimulationTransform import SimulationTransform
from utils.DashboardWebsocket import DashboardWebsocket
from colav.ColavManager import ColavManager 
from loggers.FastLogger import FastLogger 
from simulation.SimulationServer import SimulationServer

class Simulation4DOF(SimulationServer):
    def __init__(self, buffer_size, address= None,
                websocket = DashboardWebsocket, transform=SimulationTransform(), data_logger = FastLogger,
                distance_filter=None, predicted_interval = 30 ,colav_manager = ColavManager,
                filt_order = 3, filt_cutfreq = 0.1, filt_nyqfreq = 0.5, tmax = 1, dt=0.1, rvg_init={},
                send_msg_filter = ['!AI']):
        
        super(Simulation4DOF, self).__init__(buffer_size, data_logger, address,
                websocket, transform, distance_filter, predicted_interval ,colav_manager,
                filt_order, filt_cutfreq, filt_nyqfreq)  
        self._send_msg_filter = send_msg_filter
        self._running = False   
        self.ais_history = dict()
        self.ais_history_len = 30 
        self.rvg_init = rvg_init
        self.parV, self.parA = model.DefaultModelData()  
        self.tmax=tmax
        self.dt=dt #time step
        self.lat = rvg_init['lat']
        self.lat_dir = rvg_init['lat_dir']
        self.o_lat = self.transform.deg_2_dec(self.lat, self.lat_dir)
        self.lon = rvg_init['lon']
        self.lon_dir = rvg_init['lon_dir']
        self.o_lon = self.transform.deg_2_dec(self.lon, self.lon_dir)
        self.Uc= rvg_init['spd_over_grnd'] #current speed
        self.betac = rvg_init['true_course'] * math.pi/180 #current direction
        self.azi = rvg_init['azi_d']
        self.v = rvg_init['spd_over_grnd']
        self.revs = rvg_init['revs']
        self.azi_sat = np.pi/8 
        self.seqnum = 0
        self.state = np.zeros(8)
        self.nu=np.array([self.Uc,0,0,0]) #u v p r
        self.eta=np.array([0,0,0,self.betac ]) #x y phi psi 
        self.sim_timer = 0
        self.sim_buffer = []

    def spoof_gpgga_msg(self, timestamp, lon, lat, alt=12.6):
        lat_ddm, lat_dir = self.transform.dec_2_deg(lat, direction='lat')
        lon_ddm, lon_dir = self.transform.dec_2_deg(lon, direction='lon')
        dtime = datetime.fromtimestamp(timestamp)

        msg = {
        'timestamp': dtime.time(),
        'lat': lat_ddm,
        'lat_dir': lat_dir,
        'lon': lon_ddm,
        'lon_dir': lon_dir,
        'gps_qual': 1,
        'num_sats': '10',
        'horizontal_dil': '1.0',
        'altitude': alt,
        'altitude_units': 'M',
        'geo_sep': '41.4',
        'geo_sep_units': 'M',
        'age_gps_data': '',
        'ref_station_id': '',
        'unix_time': timestamp,
        'seq_num': self.seqnum,
        'src_id': 3,
        'src_name': '@10.0.8.10:34340',
        'message_id': '$GPGGA_ext'
        }
        self.seqnum = self.seqnum + 1
        return msg

    def spoof_gprmc(self, timestamp, lon, lat, speed, course):
        lat_ddm, lat_dir = self.transform.dec_2_deg(lat, direction='lat')
        lon_ddm, lon_dir = self.transform.dec_2_deg(lon, direction='lon')
        dtime = datetime.fromtimestamp(timestamp)
        speed = self.transform.mps_to_kn(speed)
        msg = {
        'timestamp': dtime.time(),
        'status': 'A',
        'lat': lat_ddm,
        'lat_dir': lat_dir,
        'lon': lon_ddm,
        'lon_dir': lon_dir,
        'spd_over_grnd': speed,
        'true_course': course,
        'datestamp': dtime.date(),
        'mag_variation': '4.7',
        'mag_var_dir': 'E',
        'unknown_0': 'A',
        'unknown_1': 'S',
        'unix_time': timestamp,
        'seq_num': self.seqnum,
        'src_id': 3,
        'src_name': 'a10.0.8.1',
        'message_id': '$GPRMC_ext'
        }
        self.seqnum = self.seqnum + 1
        return msg   
        
    def spoof_psimsns(self, timestamp, roll, yaw, pitch = 0):
        dtime = datetime.fromtimestamp(timestamp)
        msg = {
        'msg_type': 'SNS',
        'timestamp': dtime.time(),
        'unknown_1': '',
        'tcvr_num': '1',
        'tdcr_num': '1',
        'roll_deg': roll,
        'pitch_deg': pitch,
        'heave_m': '0.00',
        'head_deg': yaw,
        'empty_1': '',
        'unknown_2': '40',
        'unknown_3': '0.000',
        'empty_2': '',
        'checksum': 'M121',
        'unix_time': timestamp,
        'seq_num': self.seqnum,
        'src_id': 1,
        'src_name': '@10.0.8.10:39816',
        'message_id': '$PSIMSNS_ext'
        }
        self.seqnum = self.seqnum + 1
        return msg

    def convert_simulation(self, x, timestamps):
        out = []
        for i, timestamp in enumerate(timestamps):  
            if  i % int(len(timestamps)/2) == 0 :
                n, e, roll, psi, surge, sway = x[0:6, i]
                roll = roll * (180 / math.pi)
                psi = psi * (180 / math.pi)
                lat, lon = self.transform.xyz_to_coords(e, n, self.o_lat, self.o_lon)
                speed = math.sqrt(surge**2 + sway**2)
                course = psi + math.atan2(sway, surge) * (180/math.pi) 
                out.append(self.spoof_psimsns(timestamp, roll, psi))
                out.append(self.spoof_gpgga_msg(timestamp, lon, lat))
                out.append(self.spoof_gprmc(timestamp, lon, lat, speed, course))
        return out

    def run_simulation(self):
        tvec = np.linspace(0,self.tmax,int(self.tmax/self.dt)+1)

        #dict containing simulation param
        parS = {'dt':self.dt, 'Uc': 0, 'betac':  0} 
        # =============================================================================
        # Initial conditions and allocate memory
        # =============================================================================
        azi = self.azi * math.pi/180
        thrust_state = np.array([azi,self.revs]) #azimuth angle and revs 
        x=np.concatenate((self.eta, self.nu, thrust_state)) 
        x_out=np.zeros([len(x),len(tvec)])
        simulationStart = time() 
        for i1, t in enumerate(tvec):
            #store results 
            x_out[:,i1] = x 
            u = thrust_state
            #time integration
            Fw = np.zeros(4)  
            x = model.int_RVGMan4(x, u, Fw, self.parV, self.parA, parS) 
        timestamps = tvec + simulationStart
        #sort output 
        out=x_out[0:6,:]
        self.eta = x[0:4] 
        self.nu = x[4:8]
        return out, timestamps

    def check_incoming_controls(self):
        if 'control_azi' in self.websocket.received_data:
            self.azi = self.websocket.received_data['control_azi']
        if 'control_thrust' in self.websocket.received_data:
            self.revs = self.websocket.received_data['control_thrust']

    def start(self):
        self._running = True 
        print("Simulation 4DOF Client running...") 
        
        while self._running:
            self.check_incoming_controls()
            if len(self._buffer): #check if rt data should be sent
                for filter in self._send_msg_filter:
                    if self._buffer[0]["message_id"].find(filter) == 0:
                        self._send(self._buffer[0])
                self.pop_buffer(0)

            if (time() > self.sim_timer + self.tmax): #get 4dof sim data 
                out, timestamps =  self.run_simulation()
                self.sim_timer = timestamps[0]
                self.sim_buffer = self.convert_simulation(out, timestamps) 
            
            #send 4dof sim data for rvg
            if ( len(self.sim_buffer) and time() > self.sim_buffer[0]["unix_time"]):
                self._send(self.sim_buffer[0]) 
                self.sim_buffer.pop(0)
