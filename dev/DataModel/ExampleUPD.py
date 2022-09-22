from asyncio.windows_events import NULL 
import socket
import time
import pynmea2
from pyais import decode as ais_decode

eol_separator = '\r\n'
msg_begin_identifiers = ['!', '$']

#[['bad_1','good_1'],['bad_2','good_2'],..., ['bad_n','good_n']]
bad_eol_separators = [['\\r','\r'],['\\n', '\n']] 

# this variable sets the limit for recursive iteration loops on parse
loop_limit = 1

# Variables for logging the UDP stream
log_file_name = "datstream_5min.txt"
seconds = 300
timeout = time.time() + seconds 
log_stream = False  

#variables for identifying messages 
parsed_msg_tags = []
unknown_msg_tags = []

#variables for console output   
tag_verbose = True
unparsed_tag_verbose = True
parsed_message_verbose = False
parse_error_verbose = False

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("fagitrelay.it.ntnu.no",25508))

def save_individual_tags(raw_msg, target_list, target_list_name, parsed_message = NULL, verbose = False):
    tag = "encoded_byte_array"
    decoded_msg = raw_msg.decode(encoding='ascii')
    decoded_msg = decoded_msg.replace(eol_separator, '') 
    for begin_identifier in msg_begin_identifiers:
        if decoded_msg[0] == begin_identifier:
            tag = decoded_msg.split(',')[0] 
    if target_list.count(tag) == 0:
        target_list.append(tag)
        if verbose:
            print('\r\n tag {} for {} list:'.format(tag , target_list_name))
            print('{} \r\n has been added for message:'.format(repr(target_list)))
            print(raw_msg)
            if parsed_message is not NULL:
                print('saved parsed message is: {} \r\n'.format(repr(parsed_message)))

def update_data_object(parsed_msg, what, verbose = False):
    #update this method in order to save incoming data into a Singleton
    #also create singleton
    if verbose:
        print('type {}{} Message: {}'.format(type(parsed_msg), what, repr(parsed_msg)))
    return

def replace_bad_eol(raw_msg, eol_separator, bad_eol_separators):
    parsed_string = raw_msg.decode(encoding='ascii') 

    # There's probably a better way to handle the bad double backslash
    for separator in bad_eol_separators:
        bad_separator = separator[0]
        good_separator = separator[1]
        while parsed_string.find(bad_separator) != -1: 
            parsed_string = parsed_string.replace(bad_separator, good_separator)
    string_list = parsed_string.strip().split(eol_separator)   
    return string_list

def parse_nmea(raw_msg):
    decoded_msg = raw_msg.decode(encoding='ascii')
    parsed_msg = pynmea2.parse(decoded_msg)  
    update_data_object(parsed_msg, 'NMEA', parsed_message_verbose)
    save_individual_tags(raw_msg, parsed_msg_tags, "Succesfully Parsed", parsed_msg, tag_verbose)

def parse_ais(raw_msg):
    parsed_msg = ais_decode(raw_msg)
    update_data_object(parsed_msg, 'AIS', parsed_message_verbose)
    save_individual_tags(raw_msg, parsed_msg_tags, "Parse Success", parsed_msg, tag_verbose)

def parse_bad_eol(raw_msg, loop_limit, __loop_count):
    assert(__loop_count < loop_limit) 
    __loop_count += 1
    string_list = replace_bad_eol(raw_msg, eol_separator, bad_eol_separators) 
    assert(len(string_list) > 1)
    for entry in string_list:
        entry = entry + eol_separator 
        entry_bin = entry.encode(encoding="ascii")
        parse_message(raw_msg = entry_bin, loop_limit = loop_limit, __loop_count = __loop_count)

def parse_message(raw_msg, loop_limit = 1, __loop_count = 0):    
    try: parse_nmea(raw_msg)
    except:
        try: parse_ais(raw_msg)
        except:
            try: parse_bad_eol(raw_msg, loop_limit, __loop_count)                
            except:
                try: 
                    save_individual_tags(raw_msg, unknown_msg_tags, "Parse Failed", verbose = unparsed_tag_verbose)
                    if parse_error_verbose:
                        print('Unable to parse message: {}'.format(raw_msg))  
                except:
                    if parse_error_verbose:
                        print('Unable to parse or save message tag: {}'.format(raw_msg))
    
if log_stream:
    f = open(log_file_name, "a")

while True:
    raw_msg = s.recv(4096)  
    parse_message(raw_msg, loop_limit) 
    if log_stream:
        f.write(raw_msg)
    if time.time() > timeout and log_stream:
        f.close
        break