import websocket  

class DashboardWebsocket:
    def __init__(self, address, enable=True):
        self.address = address
        self.enable = enable

        if enable:
            websocket.enableTrace(False) 
            self.ws = websocket.create_connection(address) 
    
    def send(self, json_msg):
        if self.enable:
            self.ws.send(json_msg)

    def close(self):
        if self.enable: 
            self.ws.close()
