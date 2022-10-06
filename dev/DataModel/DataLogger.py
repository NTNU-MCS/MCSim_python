import pandas as pd
import numpy as np

class SortedData:  
    def __init__(self) -> None:
        pass

class DataLogger:
    def __init__(self, stream_parser):
        self._stream_parser = stream_parser
        self.sorted_data = SortedData
        self._running = False
        self._buffer_data = stream_parser.parsed_msg_list

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
            for idx, value in enumerate(msg_values):
                dtypes.append((msg_atr[idx], str)) #Change this to actual value type

        dtypes = np.dtype(dtypes) 
        df = pd.DataFrame(np.empty(0, dtype=dtypes)) 
        setattr(SortedData, msg_id, df)
        print(getattr(self.sorted_data, msg_id))

    def _log_nmea_data(self, message):
        msg_id, msg_atr, msg_values, unkown_msg_data = message
        df = pd.DataFrame({msg_atr[i]: [msg_values[i]] for i in range(len(msg_values))})
        nmea_data = getattr(self.sorted_data, msg_id)   
        nmea_data = pd.concat([nmea_data, df], ignore_index=True)  
        setattr(self.sorted_data, msg_id, nmea_data)  
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
        #print(msg_id, msg_atr, msg_values, unkown_msg_data)
        self._stream_parser.pop_parsed_msg_list()

    def sort_buffered_data(self):
        self._running = True

        while self._running: #should make this consistent with other classes
            self._log_buffered_message()
