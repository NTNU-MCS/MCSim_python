import websocket  
import json

class DashboardWebsocket:
    def __init__(self, address, enable=True, 
                receive_filters = ['control_azi', 'control_thrust', 'data_mode']):
        
        self.address = address
        self.enable = enable
        self.received_data = {}
        self._receive_filters = receive_filters
        self.running = False

        if enable:
            websocket.enableTrace(False) 
            self.ws = websocket.create_connection(address) 
    
    def send(self, json_msg):
        if self.enable:
            self.ws.send(json_msg)

    def recieve(self):
        return self.ws.recv()
    
    def start(self):
        self.running = True

        while self.running:
            raw = self.ws.recv()
            msg = json.loads(raw)

            if msg['type'] == 'datain': 
                msg_id = msg['data']['message_id']

                for filter in self._receive_filters:
                    if msg_id == filter:
                        val = msg['data']['val']
                        self.received_data[msg_id] = val 

    def close(self):
        self.running = False
        if self.enable: 
            self.ws.close()
