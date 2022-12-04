from threading import Thread
from StreamParser import StreamParser
from LogParser import LogParser
from DataLogger import DataLogger  
from SimulationClient import SimulationClient
from SimulationTransform import SimulationTransform
from time import sleep 
import pathlib
from datetime import datetime
import os
from Decrypter import Decrypter
import socket
import easygui

abs_path = pathlib.Path(__file__).parent.resolve()
#global: fagitrelay.it.ntnu.no
#local: gunnerus.local
address = ("fagitrelay.it.ntnu.no", 25508)
buffer_size = 4096
loop_limit = 1

#raw_verbose, tag_verbose, unparsed_tag_verbose,
#parsed_message_verbose, parse_error_verbose
verbosity = (False, False, False, False, False)
now = datetime.now()
date_time = now.strftime("%m%d%y_%H-%M-%S")
save_logs = False
log_time = 60
log_name = './datastream_'+ date_time + '_' + str(log_time) + 's.txt'
log_path = os.path.join(abs_path, 'DataStreams', log_name)
log_stream = (log_path, log_time, save_logs)

key_path = os.path.join(abs_path, 'nmeatools')
UDP_Decrypter = Decrypter(key_path = key_path)

# if True a log can be selected and used as the data source
parse_saved_log = False

if parse_saved_log:
    load_path = easygui.fileopenbox()
    UDP_Stream = LogParser(
        path = load_path,
        verbosity=verbosity, 
        decrypter=UDP_Decrypter)
else:
    UDP_Stream = StreamParser(
        address=address,
        buffer_size=buffer_size,
        verbosity=verbosity,
        log_stream=log_stream,
        decrypter=UDP_Decrypter)

simulation_origo_offset = ( 
    10.3929167,   #lon offset
    63.435166667, #lat offset
    0,            #altitude offset
    0             #heading offset
    )

UDP_Sim_Frame_transform = SimulationTransform(
offsets=simulation_origo_offset
)

headers_path = os.path.join(abs_path, './DataFrames/headers')
save_headers = (True, headers_path)
df_path = os.path.join(abs_path, './DataFrames')

# ToDo: create class for holding these tuples
df_aliases = [
    ('$PSIMSNS',['msg_type', 'timestamp', 'unknown_1', 'tcvr_num', 'tdcr_num', 'roll_deg', 'pitch_deg', 'heave_m', 'head_deg', 'empty_1', 'unknown_2', 'unknown_3', 'empty_2', 'checksum']),
    ('$PSIMSNS_ext',['msg_type', 'timestamp', 'unknown_1', 'tcvr_num', 'tdcr_num', 'roll_deg', 'pitch_deg', 'heave_m', 'head_deg', 'empty_1', 'unknown_2', 'unknown_3', 'empty_2', 'checksum']),
]

save_dataframes = (True, df_path)
overwrite_headers = True
dl_verbose = (False, parse_saved_log)

UDP_DataLogger = DataLogger(
    stream_parser=UDP_Stream,
    save_headers=save_headers,
    save_dataframes=save_dataframes,
    df_aliases=df_aliases,
    overwrite_headers=overwrite_headers,
    frame_transform=UDP_Sim_Frame_transform,
    verbose=dl_verbose
    )

sc_address = (socket.gethostname(), 5005)
sc_buffer_sz = 1024

UDP_SimulationClient = SimulationClient(
    address=sc_address, 
    buffer_size=sc_buffer_sz,
    data_logger=UDP_DataLogger,
    sim_transform=UDP_Sim_Frame_transform)

# Create new threads
thread_udp_stream = Thread(target=UDP_Stream.start) 
thread_log_data = Thread(target=UDP_DataLogger.start)
thread_sim_client = Thread(target=UDP_SimulationClient.start) 

thread_udp_stream.start()
thread_log_data.start()
thread_sim_client.start()

try:
    # wait around, catch keyboard interrupt  
    while True: 
        sleep(0.5)

except KeyboardInterrupt:
    # terminate main thread 
    UDP_Stream.stop()
    UDP_DataLogger.stop() 
    UDP_SimulationClient.stop()
    print('Exiting...')
