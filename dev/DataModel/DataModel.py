from threading import Thread
from parsers.StreamParser import StreamParser
from parsers.LogParser import LogParser
from loggers.FastLogger import FastLogger
from loggers.DataLogger import DataLogger   
from simulation.SimulationTransform import SimulationTransform
from colav.ColavManager import ColavManager
from simulation.SimulationManager import SimulationManager
import pathlib
from utils.DashboardWebsocket import DashboardWebsocket
from datetime import datetime
import os
from utils.Decrypter import Decrypter
import socket
import easygui

class DataModel:
    def __init__(self, colav_manager = ColavManager, websocket = DashboardWebsocket ,log_file=None):
        #Todo: add flush() functionality across related classes to purge data 
        #and prevent stack overflow / slowdown over extended use 
        self.abs_path = pathlib.Path(__file__).parent.resolve()

        #Settings/address for gunnerus datastream
        #global: fagitrelay.it.ntnu.no
        #local: gunnerus.local
        self.address = ("fagitrelay.it.ntnu.no", 25508)
        self.buffer_size = 4096
        self.loop_limit = 1 
        self.log_file = log_file
        #raw_verbose, tag_verbose, unparsed_tag_verbose,
        #parsed_message_verbose, parse_error_verbose
        self.verbosity = (False, False, False, False, False)
        self.now = datetime.now()
        self.date_time = self.now.strftime("%m%d%y_%H-%M-%S")
        self.save_logs = False
        self.log_time = 60
        self.log_name = './datastream_'+ self.date_time + '_' + str(self.log_time) + 's.txt'
        self.log_path = os.path.join(self.abs_path, 'DataStreams', self.log_name)
        self.log_stream = (self.log_path, self.log_time, self.save_logs)
        self.Colav_Manager = colav_manager
        self.key_path = os.path.join(self.abs_path, 'nmeatools')
        self.UDP_Decrypter = Decrypter(key_path = self.key_path)
        self.websocket = websocket
        # if True a log can be selected and used as the data source
        self.parse_saved_log = False 
        self.drop_ais_message = False

        # automatically true if using a log file as an arg
        if self.log_file is not None:
            self.parse_saved_log = True 
            self.load_path = self.log_file
        
        # filter for messages: mesages not including these strings are dropped
        self.prefixFilter = ['$PSIMSNS', '!AI', '$GPGGA', '$GPRMC']
        self.suffixFilter = '_ext'

        #Select and initialize parser
        if self.parse_saved_log:
            if self.log_file is None:
                self.load_path = easygui.fileopenbox()

            self.UDP_Stream = LogParser(
                path = self.load_path,
                verbosity=self.verbosity, 
                decrypter=self.UDP_Decrypter,
                drop_ais_messages=self.drop_ais_message,
                prefixFilter=self.prefixFilter,
                suffixFilter=self.suffixFilter)
        else:
            self.UDP_Stream = StreamParser(
                address=self.address,
                buffer_size=self.buffer_size,
                verbosity=self.verbosity,
                log_stream=self.log_stream,
                decrypter=self.UDP_Decrypter,
                drop_ais_messages=self.drop_ais_message,
                prefixFilter=self.prefixFilter,
                suffixFilter=self.suffixFilter)
            
        #Initialize Data Logger
        self.simulation_origo_offset = ( 
            10.3929167,   #lon offset
            63.435166667, #lat offset
            12,           #altitude offset
            0, #-90          #heading offset
            0,           #heading dir 
            )

        self.UDP_Sim_Frame_transform = SimulationTransform(
        offsets=self.simulation_origo_offset,
        join_type='' #'merge' merges frames on unix time, else concatenates
        )

        self.headers_path = os.path.join(self.abs_path, './DataFrames/headers')
        self.save_headers = (True, self.headers_path)
        self.df_path = os.path.join(self.abs_path, './DataFrames')

        # ToDo: create class for holding these tuples
        self.df_aliases = {
            '$PSIMSNS': ['msg_type', 'timestamp', 'unknown_1', 'tcvr_num', 'tdcr_num', 'roll_deg', 'pitch_deg', 'heave_m', 'head_deg', 'empty_1', 'unknown_2', 'unknown_3', 'empty_2', 'checksum'],
            '$PSIMSNS_ext': ['msg_type', 'timestamp', 'unknown_1', 'tcvr_num', 'tdcr_num', 'roll_deg', 'pitch_deg', 'heave_m', 'head_deg', 'empty_1', 'unknown_2', 'unknown_3', 'empty_2', 'checksum'],
        }

        self.save_dataframes = (False, self.df_path)
        self.overwrite_headers = True
        self.dl_verbose = (False, False)

        self.saveIncomingData = False

        #These should be two separate, concurrent, components.
        if (not self.saveIncomingData):
            self.UDP_DataLogger = FastLogger(
                stream_parser=self.UDP_Stream,
                save_headers=self.save_headers,
                df_aliases=self.df_aliases,
                overwrite_headers=self.overwrite_headers,
                verbose=self.dl_verbose
                )
        else:
            self.UDP_DataLogger = DataLogger(
                stream_parser=self.UDP_Stream,
                save_headers=self.save_headers,
                save_dataframes=self.save_dataframes,
                df_aliases=self.df_aliases,
                overwrite_headers=self.overwrite_headers,
                frame_transform=self.UDP_Sim_Frame_transform,
                verbose=self.dl_verbose
                )
            
        #Initialize Simulation Manager
        self.local_address = (socket.gethostname(), 5000) 
        self.sc_buffer_sz = 1024
        self.distance_filter = 1

        #dummy_gunnerus is now the origin for 4dof sim
        self.dummy_gunnerus = {
            'lat': 6326.3043,
            'lat_dir': 'E',
            'lon': 1024.5395,
            'lon_dir': 'N',
            'true_course': -40,
            'spd_over_grnd': 0,
            'revs': 100,
            'azi_d': 0,
            }
        
        self.dummy_vessel = {
            'lon': 10.411565, 
            'lat': 63.44141, 
            'course': 135,
            'heading': 190,
            'speed': 0,
            'mmsi': 3143757,
            'message_id': "!AI_ext_dummy",
            'pos_history': [[10.482652, 63.473148]],
            }
        
        self.run4DOFSim = True

        self.SimulationManager = SimulationManager(
            local_address= self.local_address,
            sc_buffer_sz = self.sc_buffer_sz,
            UDP_Stream = self.UDP_Stream,
            UDP_DataLogger = self.UDP_DataLogger,
            websocket = self.websocket,
            UDP_Sim_Frame_transform = self.UDP_Sim_Frame_transform,
            distance_filter = self.distance_filter,
            Colav_Manager = self.Colav_Manager,
            rvg_init = self.dummy_gunnerus, 
            tmax = 1, 
            dt = 0.2,
            predicted_interval = 60, 
            mode = '4dof'
            )

        # Select 'Simulation' source: saved log, 4dof, or rt
        if (self.parse_saved_log): 
            self.SimulationManager.set_simulation_type(self.SimulationManager.mode_replay)            
        elif (self.run4DOFSim): 
            self.SimulationManager.set_simulation_type(self.SimulationManager.mode_4dof)
        else:  
            self.SimulationManager.set_simulation_type(self.SimulationManager.mode_rt)
        
        # Create new threads
        self.thread_websocket_receive = Thread(target=self.websocket.start) 
        self.thread_udp_stream = Thread(target=self.UDP_Stream.start) 
        self.thread_log_data = Thread(target=self.UDP_DataLogger.start)
        self.thread_sim_manager = Thread(target=self.SimulationManager.start) 
        # self.thread_colav_manager = Thread(target=self.Colav_Manager.start)

    def start(self):
        if self.websocket.enable:
            self.thread_websocket_receive.start()
        self.thread_udp_stream.start()
        self.thread_log_data.start()
        self.thread_sim_manager.start()
        # self.thread_colav_manager.start() 

    def stop(self): 
        self.UDP_Stream.stop()
        self.UDP_DataLogger.stop() 
        self.SimulationManager.stop()
        # self.Colav_Manager.stop()
        self.websocket.close()
        print('Exiting...') 