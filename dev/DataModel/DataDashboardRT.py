
from DataModel import DataModel
from time import sleep, time
import argparse 
from multiprocessing import Process, get_context, Queue
from colav.ColavManager import ColavManager
from utils.DashboardWebsocket import DashboardWebsocket 

parser = argparse.ArgumentParser(description='File opening script')
parser.add_argument('-f', '--file', help='Path to the log file to open')
args = parser.parse_args()

ws_enable= True
ws_address= "ws://127.0.0.1:8000"
websocket = DashboardWebsocket(ws_address, ws_enable)
gunnerus_mmsi = '258342000'

#Initialize ColavManager
Colav_Manager = ColavManager(
                enable=True, 
                update_interval=10,
                websocket=websocket,
                gunnerus_mmsi = gunnerus_mmsi,
                dummy_gunnerus= None,
                dummy_vessel= None,
                safety_radius_m=200,
                safety_radius_tol=1.5,
                max_d_2_cpa=2000,
                print_comp_t=True,
                prediction_t=300
                )

#Initialize DataModel
if args.file:
        GunnerusData = DataModel(
                websocket=websocket,
                colav_manager=Colav_Manager,
                log_file=args.file
                )
else:
        GunnerusData = DataModel(
                websocket=websocket,
                colav_manager=Colav_Manager,
        )

if __name__ == '__main__':  
        process_cbf_data = Colav_Manager._cbf._process_data
        ctx = get_context('spawn')
        q = ctx.Queue() 

        try:
                GunnerusData.start() 
                Colav_Manager.start()
                
                # CBF process is created here
                while True: 
                        if time() > Colav_Manager._timeout and Colav_Manager._running:
                                start = time()
                                if (Colav_Manager.update()): 
                                        p, u, z, tq, po, zo, uo = Colav_Manager.sort_cbf_data() 

                                        #CBF computation is run in a separate process
                                        cbf_process = Process(target=process_cbf_data, args=(p, u, z, tq, po, zo, uo, q))
                                        cbf_process.start()
                                        ret_var = q.get()
                                        cbf_process.join()  
                                        Colav_Manager.send_cbf_data(ret_var) 
                                end = time()
                                
                                if Colav_Manager.print_c_time:
                                        print("end update Arpa + CBF", end-start)
                        
                        # wait around, catch keyboard interrupt 
                        sleep(0.1)

        except KeyboardInterrupt:
                # terminate main thread 
                GunnerusData.stop()
                print('Exiting...')
                