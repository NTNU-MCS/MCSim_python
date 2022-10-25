import pandas as pd
import numpy as np
from os import listdir
from os.path import isfile, join

class SortedData(object):    
    def __getitem__(self, item):
        return getattr(self, item)

class DataLogger:
    def __init__(self, stream_parser, save_headers, save_dataframes, df_aliases, overwrite_headers=False, verbose=False):
        
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
        self._verbose = verbose

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

    def _create_new_attribute(self, message):
        msg_id, msg_atr, msg_values, unkown_msg_data = message
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
        
        dtypes = np.dtype(dtypes) 
        df = pd.DataFrame(np.empty(0, dtype=dtypes)) 

        if self._save_headers:
            self._save_headers_df(msg_id, df) 
        setattr(SortedData, msg_id, df) 

    def _log_nmea_data(self, message):
        msg_id, msg_atr, msg_values, unkown_msg_data = message

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

        # ToDo: handle conversion errors better 'ignore'
        df = pd.DataFrame(
            {msg_atr[i]: [pd.to_numeric(msg_values[i], errors='ignore')] 
            for i in range(len(msg_values))}
            )

        nmea_data = getattr(self.sorted_data, msg_id)   
        nmea_data = pd.concat([nmea_data, df], ignore_index=True)  
        setattr(self.sorted_data, msg_id, nmea_data)

        if self._verbose:
            print(self.sorted_data[msg_id])

        return

    def _log_buffered_message(self):
        if len(self._buffer_data) < 1: return 
        nmea_message = self._buffer_data[-1][1]

        # ToDo: update id when new data is available from stream
        msg_id = self._buffer_data[-1][0]
        msg_atr, msg_values, unkown_msg_data = self._get_nmea_attributes(nmea_message) 
        message = (msg_id, msg_atr, msg_values, unkown_msg_data)

        if not hasattr(SortedData, msg_id):
            self._create_new_attribute(message)
        
        self._log_nmea_data(message) 
        self._stream_parser.pop_parsed_msg_list() 

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

    def _save_headers_df(self, name, df):
        file_name = self._headers_path + '/' +  name + '.pkl'
        df.to_pickle(file_name)

    def _save_dataframes(self):
        if self._save_df:
            print("Saving data...")
            for atr, df in self.sorted_data.__dict__.items(): 
                filepath = join(self._dataframes_path, atr + '.csv')
                df.to_csv(filepath)  
            print("Data Saved.")

    def stop(self):
        self._running = False

    def start(self):
        self._running = True
        print('DataLogger running.')

        while self._running: 
            self._log_buffered_message()
        self._save_dataframes()
        # ToDo: handle loose ends on terminating process.
        print('DataLogger stopped.')
