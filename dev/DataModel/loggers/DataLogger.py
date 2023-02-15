import pandas as pd
import numpy as np
from os.path import isfile, join
from loggers.Logger import Logger

class SortedData(object):    
    def __getitem__(self, item):
        return getattr(self, item)

class DataLogger(Logger):
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

    def _create_new_attribute(self, message):
        msg_id, msg_atr, msg_values, unkown_msg_data, metadata = message
        dtypes = []  

        if len(msg_atr) > 1: 
            for i, value in enumerate(msg_values):

                # ToDo: handle conversion errors better 'ignore'
                value = pd.to_numeric(value, errors='ignore')
                dtypes.append((msg_atr[i], type(value))) 
        
        if len(unkown_msg_data) > 1: 
            for i, value in enumerate(unkown_msg_data):

                # ToDo: handle conversion errors better 'ignore'
                value = pd.to_numeric(value, errors='ignore')
                atr_name = self.def_unk_atr_name + str(i)
                dtypes.append((atr_name, type(value)))  
        try: 
            alias_list = dict(self.df_aliases)[msg_id]
            if len(alias_list) == len(dtypes):
                for i , item in enumerate(dtypes): 
                    dtypes[i] = (alias_list[i], dtypes[i][1])  
        except: 
            pass

        if metadata is not None: 
            for i, value in enumerate(metadata):

                # ToDo: handle conversion errors better 'ignore'
                value = pd.to_numeric(value, errors='ignore')
                atr_name = self.metadata_atr_names[i]
                dtypes.append((atr_name, type(value)))  
        
        dtypes = np.dtype(dtypes) 
        df = pd.DataFrame(np.empty(0, dtype=dtypes)) 

        if self._save_headers:
            self._save_headers_df(msg_id, df) 
        setattr(SortedData, msg_id, df) 

    def _log_nmea_data(self, message):
        msg_id, msg_atr, msg_values, unkown_msg_data, metadata = message

        # ToDo: Probably very inefficient
        if len(unkown_msg_data) > 1: 
            msg_values.extend(unkown_msg_data)

            for i, _ in enumerate(unkown_msg_data):
                atr_name = self.def_unk_atr_name + str(i)
                msg_atr.append(atr_name)   

        # ToDo: awful, inefficient, do better check and skip redundancy
        try: 
            alias_list = dict(self.df_aliases)[msg_id] 
            if len(alias_list) == len(msg_atr):
                for i , item in enumerate(msg_atr): 
                    msg_atr[i] = alias_list[i]
        except: 
            pass

        # ToDo: Probably very inefficient x2
        if metadata is not None: 
            msg_values.extend(metadata)
            msg_atr.extend(self.metadata_atr_names)

        # ToDo: handle conversion errors better 'ignore'
        df = pd.DataFrame(
            {msg_atr[i]: [pd.to_numeric(msg_values[i], errors='ignore')] 
            for i in range(len(msg_values))}
            )

        nmea_data = getattr(self.sorted_data, msg_id)   
        nmea_data = pd.concat([nmea_data, df], ignore_index=True)  
        setattr(self.sorted_data, msg_id, nmea_data)

        if self._log_verbose:
            print(self.sorted_data[msg_id])

        return

    def _log_buffered_message(self):
        if len(self._buffer_data) < 1: 
            if self._buffer_verbose:  print('Buffer Empty')
            return  

        nmea_message = self._buffer_data[-1][1] 
        msg_id = self._buffer_data[-1][0]
        msg_atr, msg_values, unkown_msg_data = self._get_nmea_attributes(nmea_message)  

        if len(self._buffer_data[-1]) is 3:
            metadata = self._buffer_data[-1][2]
        else:
            metadata = None

        message = (msg_id, msg_atr, msg_values, unkown_msg_data, metadata)

        if not hasattr(SortedData, msg_id):
            self._create_new_attribute(message)
        
        self._log_nmea_data(message) 
        self._stream_parser.pop_parsed_msg_list() 

    def _save_headers_df(self, name, df):
        file_name = self._headers_path + '/' +  name + '.pkl'
        df.to_pickle(file_name)

    def _save_dataframes(self):
        if self._save_df:

            if self._frame_transform is not None:
                if hasattr(self.sorted_data, '$GPGGA_ext') and hasattr(self.sorted_data, '$PSIMSNS_ext'): 
                    self._frame_transform.gps_data = self.sorted_data['$GPGGA_ext']
                    self._frame_transform.attitude_data = self.sorted_data['$PSIMSNS_ext']
                    setattr(self.sorted_data, self.simulation_data_name, self._frame_transform.get_converted_frame()) 

            print("Saving data...")
            for atr, df in self.sorted_data.__dict__.items(): 
                filepath = join(self._dataframes_path, atr + '.csv')
                df.to_csv(filepath, index=False)  
            print("Data Saved.")

    def start(self):
        self._running = True
        print('DataLogger running.')

        while self._running: 
            self._log_buffered_message()
        self._save_dataframes()
        # ToDo: handle loose ends on terminating process.
        print('DataLogger stopped.')
