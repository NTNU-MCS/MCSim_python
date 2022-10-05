import socket
import time 
import sys
import pynmea2 
from pyais import decode as ais_decode

class StreamParser:
    def __init__(self, address, buffer_size, loop_limit = 1, 
                verbosity = (False, False, False, False, False), 
                log_stream = ("datstream_5min.txt", 300, False)):
        
        # method for capping the size of this object might be necessary
        # that or figure out how to throw it to the heap
        self.parsed_msg_list = []

        # keep track of the buffered messages in bytes, doesnt
        # seem to grow at a concerning rate 
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
        self.__seconds = log_stream[1]
        self.__timeout = time.time() + self.__seconds 
        self.__log_stream = log_stream[2]  

        self.__address = address 
        self.__buffer_size = buffer_size
        
        # This variable sets the limit for recursive iteration loops on parse
        self.__loop_limit = loop_limit

        # Variables for console output   
        self.__raw_verbose = verbosity[0]
        self.__tag_verbose = verbosity[1]
        self.__unparsed_tag_verbose = verbosity[2]
        self.__parsed_message_verbose = verbosity[3]
        self.__parse_error_verbose = verbosity[4]

        self.__s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__s.connect(self.__address)
    
    def __get_tag(self, raw_msg):
        tag = "unknown"
        decoded_msg = raw_msg.decode(encoding='ascii')
        decoded_msg = decoded_msg.replace(self.eol_separator, '') 

        for begin_identifier in self.msg_begin_identifiers:
            if decoded_msg[0] == begin_identifier:
                tag = decoded_msg.split(',')[0]  
        return tag
    
    def __save_individual_tags(self, raw_msg, target_list, target_list_name, parsed_message = None, verbose = False):
        tag = self.__get_tag(raw_msg)

        if target_list.count(tag) == 0:
            target_list.append(tag)
            if verbose:
                print('\r\n tag {} for {} list:'.format(tag , target_list_name))
                print('{} \r\n has been added for message:'.format(repr(target_list)))
                print(raw_msg)
                if parsed_message is not None:
                    print('saved parsed message is: {} \r\n'.format(repr(parsed_message)))

    def __update_data_object(self, parsed_msg, raw_msg, what, verbose = False): 
        #update this when extra information is added to udp message
        tag = self.__get_tag(raw_msg) 
        self.parsed_msg_list.insert(0, (tag, parsed_msg)) 
        self.parsed_msg_list_size = sys.getsizeof(self.parsed_msg_list)
        if verbose:
            print('type {}{} Message: {}'.format(type(parsed_msg), what, repr(parsed_msg)))
        return

    def __fix_bad_eol(self, raw_msg): 
        parsed_string = raw_msg.decode(encoding='ascii') 

        # There's probably a better way to handle the bad double backslash
        for separator in self.bad_eol_separators:
            bad_separator = separator[0]
            good_separator = separator[1]
            while parsed_string.find(bad_separator) != -1: 
                parsed_string = parsed_string.replace(bad_separator, good_separator)
        string_list = parsed_string.strip().split(self.eol_separator)   
        return string_list

    def __fix_collated_msgs(self, raw_msg): 
        parsed_string = raw_msg.decode(encoding='ascii') 
        for begin_identifier in self.msg_begin_identifiers:
            parsed_string = parsed_string.replace(begin_identifier, self.eol_separator + begin_identifier)

        string_list = parsed_string.strip().split(self.eol_separator)  
        return string_list

    def __parse_nmea(self, raw_msg):
        decoded_msg = raw_msg.decode(encoding='ascii')
        parsed_msg = pynmea2.parse(decoded_msg)  
        self.__update_data_object(parsed_msg, raw_msg, 'NMEA', self.__parsed_message_verbose)
        self.__save_individual_tags(raw_msg, self.parsed_msg_tags, "Succesfully Parsed", parsed_msg, self.__tag_verbose)

    def __parse_ais(self, raw_msg):
        parsed_msg = ais_decode(raw_msg)
        self.__update_data_object(parsed_msg, raw_msg, 'AIS', self.__parsed_message_verbose)
        self.__save_individual_tags(raw_msg, self.parsed_msg_tags, "Succesfully Parsed", parsed_msg, self.__tag_verbose)

    def __parse_list(self, raw_msg, list_callback, __loop_count):
        assert(__loop_count < self.__loop_limit)  
        string_list = list_callback(raw_msg) 
        assert(len(string_list) > 1)
        __loop_count += 1 
        for entry in string_list: 
            entry = entry + self.eol_separator  
            entry_bin = entry.encode(encoding="ascii") 
            self.__parse_message(entry_bin, __loop_count)
        return __loop_count

    def __parse_message(self, raw_msg, __loop_count = 0):    
        try: self.__parse_nmea(raw_msg)
        except:
            try: self.__parse_ais(raw_msg)
            except:
                try: __loop_count = self.__parse_list(raw_msg, self.__fix_bad_eol, __loop_count)                
                except:
                    try: __loop_count = self.__parse_list(raw_msg, self.__fix_collated_msgs, __loop_count)
                    except:
                        try:
                            self.__save_individual_tags(raw_msg, self.unknown_msg_tags, "Parse Failed", verbose = self.__unparsed_tag_verbose)
                            if self.__parse_error_verbose:
                                print('Unable to parse message: {}'.format(raw_msg))  
                        except:
                            if self.__parse_error_verbose:
                                print('Unable to parse or save message tag: {}'.format(raw_msg))

    def stream_udp_data(self):
        if self.__log_stream:
            f = open(self.log_file_name, "a")

        while True:
            raw_msg = self.__s.recv(self.__buffer_size)

            if self.__raw_verbose:
                print(raw_msg)

            self.__parse_message(raw_msg) 
            if self.__log_stream:
                f.write(raw_msg)
            if time.time() > self.__timeout and self.__log_stream:
                f.close
                break
