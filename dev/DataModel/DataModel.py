from threading import Thread
from StreamParser import StreamParser
import sys
from time import sleep
from _thread import interrupt_main

address = ("fagitrelay.it.ntnu.no",25508)
buffer_size = 4096
loop_limit = 1

#raw_verbose, tag_verbose, unparsed_tag_verbose,
#parsed_message_verbose, parse_error_verbose
verbosity = (False, False, False, False, True)
log_stream = ("datstream_5min.txt", 300, False)

UDP_Stream = StreamParser(address=address, buffer_size=buffer_size, verbosity=verbosity)

def check_dm_data():
    while True:
        sleep(0.5)
        print(len(UDP_Stream.parsed_msg_list))

# # Create new threads
thread_udp_stream = Thread(target=UDP_Stream.stream_udp_data)
thread_udp_stream.setDaemon(True)
thread_check_data = Thread(target=check_dm_data)
thread_check_data.setDaemon(True)

thread_udp_stream.start()
thread_check_data.start()

try:
    # wait around
    while True:
        print('Main thread waiting...')
        sleep(0.5)
except KeyboardInterrupt:
    # terminate main thread
    print('Main interrupted! Exiting.')
    interrupt_main()
    sys.exit()