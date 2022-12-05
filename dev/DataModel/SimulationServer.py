import socket
import pandas as pd

class SimulationServer:
    def __init__(self, address, buffer_size, data_logger, sim_transform):
        self._ip = address[0]
        self._port = address[1]
        self.buffer_size = buffer_size
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._data_logger = data_logger
        self._running = False
        self._sim_tf = sim_transform
        self._message_df = None
        self._index = 0
        self._attitude_df_len = 0
        self._pos_df_len = 0
        self._message_df_len = 0

    def stop(self):
        self._running = False
        print('Simulation Client stopped')

    def _send(self, message):
        message = message.encode('ascii')
        self.server.sendto(message, (self._ip, self._port))

    def _create_package(self):
        data_avail = (
            hasattr(self._data_logger.sorted_data, '$GPGGA_ext') 
            and hasattr(self._data_logger.sorted_data, '$PSIMSNS_ext'))
        
        if data_avail: 
            frames_updated = (
                self._attitude_df_len < self._data_logger.sorted_data['$PSIMSNS_ext'].shape[0] and 
                self._pos_df_len < self._data_logger.sorted_data['$GPGGA_ext'].shape[0])

            if frames_updated:
                self._sim_tf.gps_data = self._data_logger.sorted_data['$GPGGA_ext']
                self._sim_tf.attitude_data = self._data_logger.sorted_data['$PSIMSNS_ext']

                self._attitude_df_len = self._data_logger.sorted_data['$PSIMSNS_ext'].shape[0]
                self._pos_df_len = self._data_logger.sorted_data['$GPGGA_ext'].shape[0]

                self._message_df = self._sim_tf.get_converted_frame()
                self._message_df_len = self._message_df.shape[0]

    def start(self):
        self._running = True
        print('Simulation Client running...')
        while self._running:
            self._create_package()
            if self._index < self._message_df_len:
                message = self._message_df.iloc[self._index].tolist()
                message = ','.join(message)
                self._send(message)
                self._index += 1
