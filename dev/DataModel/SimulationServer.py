import socket
import pandas as pd
import websocket 
import json
from SimulationTransform import SimulationTransform


class SimulationServer:
    def __init__(self, buffer_size, data_logger, address= None,
                ws_enable=False, ws_address='', transform=SimulationTransform()):
        
        self._buffer = data_logger.sorted_data
        self._running = False
        self.ws_enable = ws_enable
        self.address = address
        self.transform = transform
        
        if ws_enable:
            websocket.enableTrace(True)
            self._ws_address = ws_address
            self.ws = websocket.create_connection(self._ws_address)
        if address is not None:
            self._ip = address[0]
            self._port = address[1]
            self.buffer_size = buffer_size
            self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def stop(self):
        self._running = False
        print('Simulation Client stopped')

    def _compose_msg(self, msg, msg_type = 'datain'): 
        return(json.dumps({
            "type": msg_type,
            "content": msg
            },default=str))
    
    def _compose_transformed_msg(self, msg):
        northings = self.transform.deg_2_dec(msg['lat'], msg['lat_dir'])
        eastings = self.transform.deg_2_dec(msg['lon'], msg['lon_dir'])
        altitude = msg['altitude']
        x,y,z = self.transform.get_xyz(northings, eastings, altitude)
        return(json.dumps({"message_id":"coords","x":x, "y":y, "z":z}, default=str))

    def _send(self, message):
        json_msg = self._compose_msg(message)

        if self.ws_enable: 
            self.ws.send(json_msg)
        if self.address is not None:
            if message["message_id"]=="$GPGGA_ext": 
                msg = self._compose_transformed_msg(message)
                msg = msg.encode('ascii')
                self.server.sendto(msg, (self._ip, self._port))
            if message["message_id"]=="$PSIMSNS_ext":
                msg = json.dumps(message, default=str)
                msg = msg.encode('ascii')
                self.server.sendto(msg, (self._ip, self._port))

    def pop_buffer(self, index = None):
        if len(self._buffer) < 1: return

        if index is not None:
            return self._buffer.pop(index)
        else:
            return self._buffer.pop()
        
    def start(self):
        self._running = True 
        print('Simulation Client running...')

        while self._running:
            if len(self._buffer):
                self._send(self._buffer[0])
                self.pop_buffer(0)

        self.ws.close()
