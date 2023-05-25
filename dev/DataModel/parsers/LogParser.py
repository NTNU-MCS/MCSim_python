import time 
import sys
import select
import os
from utils.Decrypter import Decrypter
from parsers.Parser import Parser
from tqdm import tqdm

class LogParser(Parser):
    def __init__(self, path ,loop_limit = 1, 
                verbosity = (False, False, False, False, False), 
                decrypter = Decrypter, drop_ais_messages = True,
                prefixFilter = [], suffixFilter=''
                ):
        
        self.parse_complete = False
        self.prefixFilter = prefixFilter
        self.suffixFilter = suffixFilter
        self.extended_msg_suffix  = '_ext'
        # decrypter
        self._decrypter = decrypter
        
        # method for capping the size of this object might be necessary
        # that or figure out how to throw it to the heap
        self.parsed_msg_list = []

        #incoming ais messages will be ignored if True
        self.drop_ais_messages = drop_ais_messages
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
        
        # This variable sets the limit for recursive iteration loops on parse
        self._loop_limit = loop_limit

        # Variables for console output   
        self._raw_verbose = verbosity[0]
        self._tag_verbose = verbosity[1]
        self._unparsed_tag_verbose = verbosity[2]
        self._parsed_message_verbose = verbosity[3]
        self._parse_error_verbose = verbosity[4]

        self.path = path

         # Variables for AIS Decoding
        self.talker = ['!AIVDM', '!AIVDO']
        self.max_id = 10
        self._buffer = [None] * self.max_id

    def start(self):
        print("StreamParser running.")
        self._running = True
        file = open(self.path , 'r')
        lines = file.readlines()  
        for line in tqdm(lines):
            if not self._running: return
            raw_msg = line.encode(encoding='ascii')
            if self._raw_verbose: print(raw_msg)
            self._parse_message(raw_msg)  
        self.parse_complete = True

        print("StreamParser Stopped.")           
