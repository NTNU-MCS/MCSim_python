from threading import Thread
from parsers.StreamParser import StreamParser
from parsers.LogParser import LogParser
from loggers.DataLogger import DataLogger  
from loggers.SimulationLogger import SimulationLogger
from simulation.SimulationServer import SimulationServer
from simulation.SimulationTransform import SimulationTransform
from colav.ColavManager import ColavManager
import pathlib
from utils.DashboardWebsocket import DashboardWebsocket
from datetime import datetime
import os
from utils.Decrypter import Decrypter
import socket
import easygui

class DataModel:
    def __init__(self):
        #Todo: add flush() functionality across related classes to purge data 
        #and prevent stack overflow / slowdown over extended use 
        self.gunnerus_mmsi = '258342000'
        self.abs_path = pathlib.Path(__file__).parent.resolve()
        #global: fagitrelay.it.ntnu.no
        #local: gunnerus.local
        self.address = ("fagitrelay.it.ntnu.no", 25508)
        self.buffer_size = 4096
        self.loop_limit = 1

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

        self.key_path = os.path.join(self.abs_path, 'nmeatools')
        self.UDP_Decrypter = Decrypter(key_path = self.key_path)

        # if True a log can be selected and used as the data source
        self.parse_saved_log = False
        self.drop_ais_message = False
         # filter for messages
        self.prefixFilter = ['$PSIMSNS', '!AI', '$GPGGA', '$GPRMC']
        self.suffixFilter = '_ext'

        if self.parse_saved_log:
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
        self.df_aliases = [
            ('$PSIMSNS',['msg_type', 'timestamp', 'unknown_1', 'tcvr_num', 'tdcr_num', 'roll_deg', 'pitch_deg', 'heave_m', 'head_deg', 'empty_1', 'unknown_2', 'unknown_3', 'empty_2', 'checksum']),
            ('$PSIMSNS_ext',['msg_type', 'timestamp', 'unknown_1', 'tcvr_num', 'tdcr_num', 'roll_deg', 'pitch_deg', 'heave_m', 'head_deg', 'empty_1', 'unknown_2', 'unknown_3', 'empty_2', 'checksum']),
        ]

        self.save_dataframes = (False, self.df_path)
        self.overwrite_headers = True
        self.dl_verbose = (False, False)

        self.feedSimulation = True

        if (self.feedSimulation):
            self.UDP_DataLogger = SimulationLogger(
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
        
        self.local_address = (socket.gethostname(), 5000) 
        self.sc_buffer_sz = 1024
        self.distance_filter = 1
        self.ws_enable= True
        self.ws_address= "ws://127.0.0.1:8000"
        self.websocket = DashboardWebsocket(self.ws_address, self.ws_enable)
        self.dummy_gunnerus = {
            'lat': 6228.4822,
            'lat_dir': 'W',
            'lon': 609.1721,
            'lon_dir': 'N',
            'true_course': 270,
            'spd_over_grnd': 100,
            }
        self.dummy_vessel = {
            'lon': 6.15405, 
            'lat': 62.473457, 
            'course': -30,
            'speed': 100,
            'mmsi': 3143757
             }


        self.Colav_Manager = ColavManager(
            enable=True, 
            update_interval=10,
            websocket=self.websocket,
            gunnerus_mmsi = self.gunnerus_mmsi,
            dummy_gunnerus= None,
            dummy_vessel= None
            )

        self.UDP_SimulationServer = SimulationServer(
            address=self.local_address, 
            buffer_size=self.sc_buffer_sz,
            data_logger=self.UDP_DataLogger,
            websocket=self.websocket,
            transform=self.UDP_Sim_Frame_transform,
            distance_filter=self.distance_filter,
            colav_manager=self.Colav_Manager
            ) 
        

        # Create new threads
        self.thread_udp_stream = Thread(target=self.UDP_Stream.start) 
        self.thread_log_data = Thread(target=self.UDP_DataLogger.start)
        self.thread_sim_server = Thread(target=self.UDP_SimulationServer.start) 
        self.thread_colav_manager = Thread(target=self.Colav_Manager.start)

    def start(self):
        self.thread_udp_stream.start()
        self.thread_log_data.start()
        self.thread_sim_server.start()
        self.thread_colav_manager.start() 

    def stop(self):
        self.UDP_Stream.stop()
        self.UDP_DataLogger.stop() 
        self.UDP_SimulationServer.stop()
        self.Colav_Manager.stop()
        self.websocket.close()
        print('Exiting...') 