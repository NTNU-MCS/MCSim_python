from simulation.SimulationServer import SimulationServer
from simulation.SimulationServerReplay import SimulationServerReplay
from simulation.Simulation4DOF import Simulation4DOF
from simulation.SimulationTransform import SimulationTransform

class SimulationManager():
    def __init__(self, local_address, sc_buffer_sz, UDP_Stream, UDP_DataLogger,
                        websocket, UDP_Sim_Frame_transform, distance_filter,
                        Colav_Manager, rvg_init, tmax = 1, dt = 0.2,
                        predicted_interval = 60, mode = '4dof'):
        
        self.local_address = local_address
        self.sc_buffer_sz = sc_buffer_sz,
        self.UDP_Stream = UDP_Stream
        self.UDP_DataLogger = UDP_DataLogger
        self.websocket = websocket
        self.UDP_Sim_Frame_transform = UDP_Sim_Frame_transform
        self.distance_filter = distance_filter
        self.Colav_Manager = Colav_Manager 
        self.rvg_init = rvg_init 
        self.tmax = tmax 
        self.dt = dt
        self.predicted_interval = predicted_interval
        self.mode_4dof = '4dof'
        self.mode_replay = 'replay'
        self.mode_rt = 'rt'
        self.mode = mode
        self.SimulationServer = SimulationServer

    def start(self):
        self.SimulationServer.start()

    def stop(self):
        self.SimulationServer.stop()
    
    def set_simulation_type(self, mode):
        if (mode == self.mode_replay): 
                self.SimulationServer = SimulationServerReplay(
                    address=self.local_address, 
                    buffer_size=self.sc_buffer_sz,
                    logParser=self.UDP_Stream,
                    data_logger=self.UDP_DataLogger,
                    websocket=self.websocket,
                    transform=self.UDP_Sim_Frame_transform,
                    distance_filter=self.distance_filter,
                    predicted_interval=self.predicted_interval,
                    colav_manager=self.Colav_Manager
                    )  
        elif (mode == self.mode_4dof): 
            self.SimulationServer = Simulation4DOF(
                buffer_size=self.sc_buffer_sz,
                address=self.local_address, 
                websocket =self.websocket,
                data_logger=self.UDP_DataLogger,
                transform=self.UDP_Sim_Frame_transform, 
                distance_filter=self.distance_filter, 
                colav_manager=self.Colav_Manager, 
                tmax = self.tmax, 
                dt=self.dt, 
                rvg_init=self.rvg_init
                )
        else:  
            self.SimulationServer = SimulationServer(
                address=self.local_address, 
                buffer_size=self.sc_buffer_sz,
                data_logger=self.UDP_DataLogger,
                websocket=self.websocket,
                transform=self.UDP_Sim_Frame_transform,
                distance_filter=self.distance_filter,
                predicted_interval=self.predicted_interval,
                colav_manager=self.Colav_Manager
                )