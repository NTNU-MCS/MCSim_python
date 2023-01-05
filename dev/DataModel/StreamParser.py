import socket
import time 
import sys
import select
import os
import Decrypter
from Parser import Parser

class StreamParser(Parser):
    def __init__(self, address, buffer_size, loop_limit = 1, 
                verbosity = (False, False, False, False, False), 
                log_stream = ("datstream_5min.txt", 300, False),
                socket_timeout = 5, decrypter = Decrypter):

        self.extended_msg_suffix  = '_ext'
        # decrypter
        self._decrypter = decrypter
        
        # method for capping the size of this object might be necessary
        # that or figure out how to throw it to the heap
        self.parsed_msg_list = []

        #incoming ais messages will be ignored if True
        self.drop_ais_messages = True
        self._running = False

        # keep track of the buffered messages in bytes, doesnt
        # seem to grow at a concerning rate if at all
        self.parsed_msg_list_size = 0

        # [['bad_1','good_1'],['bad_2','good_2'],..,['bad_n','good_n']]
        self.bad_eol_separators = [['\\r','\r'],['\\n', '\n']]
        self.eol_separator = '\r\n'
        self.msg_begin_identifiers = ['!', '$']

        # Variables for identifying messages 
        self.parsed_msg_tags = []
        self.unknown_msg_tags = []

        # Variables for logging the UDP stream
        self.log_file_name = log_stream[0]
        self._seconds = log_stream[1]
        self._timeout = 0
        self._log_stream = log_stream[2]  

        self._address = address 
        self._buffer_size = buffer_size
        
        # This variable sets the limit for recursive iteration loops on parse
        self._loop_limit = loop_limit

        # Variables for console output   
        self._raw_verbose = verbosity[0]
        self._tag_verbose = verbosity[1]
        self._unparsed_tag_verbose = verbosity[2]
        self._parsed_message_verbose = verbosity[3]
        self._parse_error_verbose = verbosity[4]

        self._socket_timeout = socket_timeout
        self._s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self._s.connect(self._address)

    def start(self):
        print("StreamParser running.")

        self._running = True

        if self._log_stream: 
            self._timeout = time.time() + self._seconds 
            f = open(self.log_file_name, "a")
            print("Opening save file:", self.log_file_name)

        ready = select.select([self._s], [], [], self._socket_timeout)

        if not ready[0]:
            print("Connection Timed out, closing...") 

        while self._running and ready[0]: 

            raw_msg = self._s.recv(self._buffer_size)

            if self._raw_verbose:
                print(raw_msg)

            self._parse_message(raw_msg) 

            if self._log_stream:
                f.write(raw_msg.decode(encoding='ascii'))
            
            if time.time() > self._timeout and self._log_stream:
                f.close()
                break
            
        # ToDo: handle loose ends on terminating process.
        if self._log_stream:
                print("writing done, closing file ", self.log_file_name )
                f.close()

        print("StreamParser Stopped.")           
