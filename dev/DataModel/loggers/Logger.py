import pandas as pd 
from os import listdir
from os.path import isfile, join

class SortedData(object):    
    def __getitem__(self, item):
        return getattr(self, item)

class Logger:
    def __init__(self, stream_parser, save_headers, save_dataframes, df_aliases, overwrite_headers=False, frame_transform=None, verbose=False):
        
        #attribute aliases for incoming messages
        self.df_aliases = df_aliases

        # define name for unknown atribute
        self.def_unk_atr_name = 'unknown_'

        self._stream_parser = stream_parser
        self.sorted_data = SortedData()
        self._running = False
        self._save_headers = save_headers[0]
        self._headers_path = save_headers[1]
        self._save_df = save_dataframes[0]
        self._dataframes_path = save_dataframes[1]         
        self._buffer_data = stream_parser.parsed_msg_list
        self._overwrite_headers = overwrite_headers
        self._log_verbose = verbose[0]
        self._buffer_verbose = verbose[1]
        self.metadata_atr_names = ('unix_time', 'seq_num', 'src_id', 'src_name')
        self._frame_transform = frame_transform
        self.simulation_data_name = "simulation_frame"

        if not self._overwrite_headers:
            self._load_headers()

    def _get_nmea_attributes(self, nmea_object):
        t = type(nmea_object)
        msg_values = []        
        msg_atr = []
        unkown_msg_data = []

        for i , v in enumerate(nmea_object.data): 
            if i >= len(t.fields):
                unkown_msg_data.append(v)
                continue
            name = t.fields[i][1]
            msg_atr.append(name)
            msg_values.append(getattr(nmea_object, name))

        return(msg_atr, msg_values, unkown_msg_data)
    
    def _get_ais_attributes(self, ais_object):  
        ais_dict = ais_object.asdict()
        msg_values = list(ais_dict.values())     
        msg_atr = list(ais_dict.keys())
        mmsi = ais_dict['mmsi'] 

        return(msg_atr, msg_values, [], mmsi)

    def _load_headers(self):
        headers = []
        names = []
        dir_list = listdir(self._headers_path)

        if len(dir_list) < 1:
            return

        print("Loading headers...")
        for name in dir_list:
            file = join(self._headers_path, name)
            if isfile(file):
                headers.append(file)
                names.append(name.split('.')[0])

        for name, file in zip(names, headers):
            df = pd.read_pickle(file)
            setattr(SortedData, name, df) 
        print("Headers Loaded.")

    def stop(self):
        self._running = False

    def start(self):
        self._running = True
        print('DataLogger running.')
