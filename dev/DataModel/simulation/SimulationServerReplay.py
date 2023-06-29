import socket
import numpy as np 
from scipy.signal import butter, filtfilt
import json
from parsers.LogParser import LogParser
from loggers.FastLogger import FastLogger 
from simulation.SimulationServer import SimulationServer
from simulation.SimulationTransform import SimulationTransform
from utils.DashboardWebsocket import DashboardWebsocket
from colav.ColavManager import ColavManager
import math
from time import time

class SimulationServerReplay(SimulationServer):
    def __init__(self, buffer_size, data_logger = FastLogger, address= None, logParser = LogParser,
                websocket = DashboardWebsocket, transform=SimulationTransform(), 
                distance_filter=None, predicted_interval = 30 ,colav_manager = ColavManager,
                filt_order = 3, filt_cutfreq = 0.1, filt_nyqfreq = 0.5):
        super(SimulationServerReplay, self).__init__(buffer_size, data_logger, address,
                websocket, transform, distance_filter, predicted_interval ,colav_manager,
                filt_order, filt_cutfreq, filt_nyqfreq)
        self.logParser = logParser
        
    def start(self):
        self._running = True 
        print("Simulation Replay Client running...")
        sortedList = None 
        index = 0
        time_of_first_message = 0
        start_time = 0
        while self._running:
            if (self.logParser.parse_complete and not self._data_logger.bufferBusy): 
                if sortedList is None: 
                    sortedList = sorted(self._buffer, key=lambda d: d['seq_num'])  
                    time_of_first_message = sortedList[index]['unix_time']
                    start_time = time() 
                else:
                    simulation_time = time_of_first_message + (time() - start_time)
                    if(simulation_time > self._buffer[index]['unix_time']):
                        self._send(self._buffer[index])
                        index += 1
                        if(index >= len(sortedList)):
                            print('loop repeat', start_time - time())
                            self.clear_ais_history()
                            index = 0
                            start_time = time()
            # self.pop_buffer(0)
