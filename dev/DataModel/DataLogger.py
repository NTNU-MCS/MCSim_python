import pandas as pd
import numpy as np

class SortedData:  
    def __init__(self) -> None:
        pass

class DataLogger:
    def __init__(self, stream_parser):
        self.def_unk_atr_name = 'unknown_'
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
            for i, value in enumerate(msg_values):
                # ToDo: change this 'str' to actual value type
                dtypes.append((msg_atr[i], str)) 
        
        if len(unkown_msg_data) > 1: 
            for i, value in enumerate(unkown_msg_data):
                atr_name = self.def_unk_atr_name + str(i)
                dtypes.append((atr_name, str))  

        dtypes = np.dtype(dtypes) 
        df = pd.DataFrame(np.empty(0, dtype=dtypes)) 
        setattr(SortedData, msg_id, df) 

    def _log_nmea_data(self, message):
        msg_id, msg_atr, msg_values, unkown_msg_data = message

        # ToDo: Probably very inefficient
        if len(unkown_msg_data) > 1: 
            msg_values.extend(unkown_msg_data)

            for i, value in enumerate(unkown_msg_data):
                atr_name = self.def_unk_atr_name + str(i)
                msg_atr.append(atr_name)  

        df = pd.DataFrame({msg_atr[i]: [msg_values[i]] for i in range(len(msg_values))})
        nmea_data = getattr(self.sorted_data, msg_id)   
        nmea_data = pd.concat([nmea_data, df], ignore_index=True)  
        setattr(self.sorted_data, msg_id, nmea_data)  
        print(getattr(self.sorted_data, msg_id))
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

    def sort_buffered_data(self):
        self._running = True

        # ToDo: should make this consistent with other classes
        while self._running: 
            self._log_buffered_message()
