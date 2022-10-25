from threading import Thread
from DataVisualizer import DataVisualizer
from StreamParser import StreamParser
from DataLogger import DataLogger 
import matplotlib.pyplot as plt
from time import sleep 
import pathlib
import os

address = ("fagitrelay.it.ntnu.no", 25508)
buffer_size = 4096
loop_limit = 1

#raw_verbose, tag_verbose, unparsed_tag_verbose,
#parsed_message_verbose, parse_error_verbose
verbosity = (False, False, False, False, False)
log_stream = ("datstream_5min.txt", 300, False)

UDP_Stream = StreamParser(
    address=address,
    buffer_size=buffer_size,
    verbosity=verbosity)

abs_path = pathlib.Path(__file__).parent.resolve()
headers_path = os.path.join(abs_path, './DataFrames/headers')
save_headers = (True, headers_path)
df_path = os.path.join(abs_path, './DataFrames')

# ToDo: create class for holding these tuples
df_aliases = [('$PSIMSNS',['msg_type', 'timestamp', 'unknown_1', 'tcvr_num', 'tdcr_num', 'roll_deg', 'pitch_deg', 'heave_m', 'head_deg', 'empty_1', 'unknown_2', 'unknown_3', 'empty_2', 'checksum'])]
save_dataframes = (True, df_path)
overwrite_headers = False

UDP_DataLogger = DataLogger(
    stream_parser=UDP_Stream,
    save_headers=save_headers,
    save_dataframes=save_dataframes,
    df_aliases=df_aliases,
    overwrite_headers=overwrite_headers
    )

# plot_info = [('$PSIMSNS', ('timestamp', 'head_deg'))]

# UDP_Visualizer = DataVisualizer(
#     data_logger=UDP_DataLogger,
#     plot_info=plot_info
# )

# Create new threads
thread_udp_stream = Thread(target=UDP_Stream.start) 
thread_log_data = Thread(target=UDP_DataLogger.start) 
#thread_visualize_data = Thread(target=UDP_Visualizer.start) 

thread_udp_stream.start()
thread_log_data.start()
#thread_visualize_data.start()

try:
    # wait around, catch keyboard interrupt  
    #plt.ion()
    while True: 
        sleep(0.5) 
        # x = UDP_DataLogger.sorted_data['$PSIMSNS']['timestamp']
        # y = UDP_DataLogger.sorted_data['$PSIMSNS']['head_deg'] 
        # plt.plot(x, y)
        # plt.title('$PSIMSNS')  
        # plt.draw()
        # plt.pause(0.1)  

except KeyboardInterrupt:
    # terminate main thread 
    UDP_Stream.stop()
    UDP_DataLogger.stop() 
    print('Exiting...')
    plt.show(block = True)
