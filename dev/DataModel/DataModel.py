from threading import Thread
from StreamParser import StreamParser
from DataLogger import DataLogger 
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

df_aliases = [('$PSIMSNS',['msg_type', 'timestamp', 'unknown_1', 'tcvr_num', 'tdcr_num', 'roll_deg', 'pitch_deg', 'heave_m', 'head_deg', 'empty_1', 'unknown_2', 'unknown_3', 'empty_2', 'checksum'])]

UDP_Stream = StreamParser(
    address=address,
    buffer_size=buffer_size,
    verbosity=verbosity)

abs_path = pathlib.Path(__file__).parent.resolve()
headers_path = os.path.join(abs_path, './DataFrames/headers')
save_headers = (True, headers_path)
df_path = os.path.join(abs_path, './DataFrames')
save_dataframes = (True, df_path)
overwrite_headers = True

UDP_DataLogger = DataLogger(
    stream_parser=UDP_Stream,
    save_headers=save_headers,
    save_dataframes=save_dataframes,
    df_aliases=df_aliases,
    overwrite_headers=overwrite_headers
    )

# # Create new threads
thread_udp_stream = Thread(target=UDP_Stream.stream_udp_data) 
thread_log_data = Thread(target=UDP_DataLogger.sort_buffered_data) 

thread_udp_stream.start()
thread_log_data.start()

try:
    # wait around
    while True: 
        sleep(0.5)
except KeyboardInterrupt:
    # terminate main thread
    UDP_Stream.stop()
    UDP_DataLogger.stop() 
    print('Exiting...')
