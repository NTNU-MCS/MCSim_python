class DataLogger:
    def __init__(self, stream_parser):
        self._stream_parser = stream_parser
        self.sorted_data = {}
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
    
    def _log_buffered_message(self):
        if len(self._buffer_data) < 1: return 
        nmea_message = self._buffer_data[-1][1]
        # ToDo: update id when new data is available from stream
        msg_id = self._buffer_data[-1][0]
        msg_atr, msg_values, unkown_msg_data = self._get_nmea_attributes(nmea_message) 
        #print(msg_id, msg_atr, msg_values, unkown_msg_data)
        self._stream_parser.pop_parsed_msg_list()

    def sort_buffered_data(self):
        self._running = True

        while self._running: #should make this consistent with other classes
            self._log_buffered_message()
