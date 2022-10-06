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

        #incoming ais messages will be ignored if True
        self.drop_ais_messages = True
        self._stop = False

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
        self._seconds = log_stream[1]
        self._timeout = time.time() + self._seconds 
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

        self._s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._s.connect(self._address)

    def pop_parsed_msg_list(self, index = None):
        if len(self.parsed_msg_list) < 1: return

        if index is not None:
            return self.parsed_msg_list.pop(index)
        else:
            return self.parsed_msg_list.pop()
    
    def stop(self):
        self._stop = True
        return

    def _get_tag(self, raw_msg):
        tag = "unknown"
        decoded_msg = raw_msg.decode(encoding='ascii')
        decoded_msg = decoded_msg.replace(self.eol_separator, '') 

        for begin_identifier in self.msg_begin_identifiers:
            if decoded_msg[0] == begin_identifier:
                tag = decoded_msg.split(',')[0]  
        return tag
    
    def _save_individual_tags(self, raw_msg, target_list, target_list_name, parsed_message = None, verbose = False):
        tag = self._get_tag(raw_msg)

        if target_list.count(tag) == 0:
            target_list.append(tag)
            if verbose:
                print('\r\n tag {} for {} list:'.format(tag , target_list_name))
                print('{} \r\n has been added for message:'.format(repr(target_list)))
                print(raw_msg)
                if parsed_message is not None:
                    print('saved parsed message is: {} \r\n'.format(repr(parsed_message)))

    def _update_data_object(self, parsed_msg, raw_msg, what, verbose = False): 
        #update this when extra information is added to udp message
        tag = self._get_tag(raw_msg) 
        self.parsed_msg_list.insert(0, (tag, parsed_msg)) 
        self.parsed_msg_list_size = sys.getsizeof(self.parsed_msg_list)
        if verbose:
            print('type {}{} Message: {}'.format(type(parsed_msg), what, repr(parsed_msg)))
        return

    def _fix_bad_eol(self, raw_msg): 
        parsed_string = raw_msg.decode(encoding='ascii') 

        # There's probably a better way to handle the bad double backslash
        for separator in self.bad_eol_separators:
            bad_separator = separator[0]
            good_separator = separator[1]
            while parsed_string.find(bad_separator) != -1: 
                parsed_string = parsed_string.replace(bad_separator, good_separator)
        string_list = parsed_string.strip().split(self.eol_separator)   
        return string_list

    def _fix_collated_msgs(self, raw_msg): 
        parsed_string = raw_msg.decode(encoding='ascii') 
        for begin_identifier in self.msg_begin_identifiers:
            parsed_string = parsed_string.replace(begin_identifier, self.eol_separator + begin_identifier)

        string_list = parsed_string.strip().split(self.eol_separator)  
        return string_list

    def _parse_nmea(self, raw_msg):
        decoded_msg = raw_msg.decode(encoding='ascii')
        parsed_msg = pynmea2.parse(decoded_msg)  
        self._update_data_object(parsed_msg, raw_msg, 'NMEA', self._parsed_message_verbose)
        self._save_individual_tags(raw_msg, self.parsed_msg_tags, "Succesfully Parsed", parsed_msg, self._tag_verbose)

    def _parse_ais(self, raw_msg):
        parsed_msg = ais_decode(raw_msg)
        
        if self.drop_ais_messages: return

        self._save_individual_tags(raw_msg, self.parsed_msg_tags, "Succesfully Parsed", parsed_msg, self._tag_verbose)
        self._update_data_object(parsed_msg, raw_msg, 'AIS', self._parsed_message_verbose)
        
    def _parse_list(self, raw_msg, list_callback, _loop_count):
        assert(_loop_count < self._loop_limit)  
        string_list = list_callback(raw_msg) 
        assert(len(string_list) > 1)
        _loop_count += 1 
        for entry in string_list: 
            entry = entry + self.eol_separator  
            entry_bin = entry.encode(encoding="ascii") 
            self._parse_message(entry_bin, _loop_count)
        return _loop_count

    def _parse_message(self, raw_msg, _loop_count = 0):    
        try: self._parse_nmea(raw_msg)
        except:
            try: self._parse_ais(raw_msg)
            except:
                try: _loop_count = self._parse_list(raw_msg, self._fix_bad_eol, _loop_count)                
                except:
                    try: _loop_count = self._parse_list(raw_msg, self._fix_collated_msgs, _loop_count)
                    except:
                        try:
                            self._save_individual_tags(raw_msg, self.unknown_msg_tags, "Parse Failed", verbose = self._unparsed_tag_verbose)
                            if self._parse_error_verbose:
                                print('Unable to parse message: {}'.format(raw_msg))  
                        except:
                            if self._parse_error_verbose:
                                print('Unable to parse or save message tag: {}'.format(raw_msg))

    def stream_udp_data(self):
        print("launching parser")
        if self._log_stream:
            f = open(self.log_file_name, "a")

        while True: 
            raw_msg = self._s.recv(self._buffer_size)

            if self._raw_verbose:
                print(raw_msg)

            self._parse_message(raw_msg) 
            if self._log_stream:
                f.write(raw_msg)
            if time.time() > self._timeout and self._log_stream:
                f.close
                break
            if self._stop: #should make this consistent with other classes
                break
