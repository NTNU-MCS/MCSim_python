from asyncio.windows_events import NULL 
import socket
import time
import pynmea2
from pyais import decode as ais_decode

eol_separator = '\r\n'

#[['bad_1','good_1'],['bad_2','good_2'],..., ['bad_n','good_n']]
bad_eol_separators = [['\\r','\r'],['\\n', '\n']] 
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
parse_error_verbose = True

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("fagitrelay.it.ntnu.no",25508))

def save_individual_tags(raw_msg, target_list, parsed_message = NULL, verbose = False):
    #Tiny  
    tag = 'unknown'
    decoded_msg = raw_msg.decode(encoding='ascii')
    decoded_msg = decoded_msg.replace(eol_separator, '')

    if decoded_msg[0] == '$' or decoded_msg[0] == '!':
        tag = decoded_msg.split(',')[0]
    else:
        tag = "encoded_byte_array"
    
    if target_list.count(tag) == 0:
        target_list.append(tag)
        if verbose:
            print('tag {} for list {} has been added for message: \r\n {}'.format(tag , repr(target_list), raw_msg))
            if parsed_message is not NULL:
                print('saved parsed message is: {}'.format(repr(parsed_message)))

def update_data_object(parsed_msg, what, verbose = False):
    #update this method in order to save incoming data into a Singleton
    #also create singleton
    if verbose:
        print('type {}{} Message: {}'.format(type(parsed_msg), what, repr(parsed_msg)))
    return

def split_collated_string(raw_msg, eol_separator, bad_eol_separators):
    parsed_string = raw_msg.decode(encoding='ascii') 

    # There's probably a better way to handle the bad double backslash
    for separator in bad_eol_separators:
        bad_separator = separator[0]
        good_separator = separator[1]

        while parsed_string.find(bad_separator) != -1: 
            parsed_string = parsed_string.replace(bad_separator, good_separator)

    string_list = parsed_string.strip().split(eol_separator)  
    return string_list

def parse_nmea_message(raw_msg, loop_limit = 1,__loop_count = 0):    
    try:  
        decoded_msg = raw_msg.decode(encoding='ascii')
        parsed_msg = pynmea2.parse(decoded_msg)  
        update_data_object(parsed_msg, 'NMEA', parsed_message_verbose)
        save_individual_tags(raw_msg, parsed_msg_tags, parsed_msg, tag_verbose)
    except pynmea2.ParseError:
        try:  
            parsed_msg = ais_decode(raw_msg)
            update_data_object(parsed_msg, 'AIS', parsed_message_verbose)
            save_individual_tags(raw_msg, parsed_msg_tags, parsed_msg, tag_verbose)
        except:
            try:
                assert(__loop_count < loop_limit)
                __loop_count += 1
                string_list = split_collated_string(raw_msg, eol_separator, bad_eol_separators)
                for entry in string_list:
                    entry = entry + eol_separator 
                    entry_bin = entry.encode(encoding="ascii")
                    parse_nmea_message(entry_bin, loop_limit, __loop_count)
            except:
                try: 
                    save_individual_tags(raw_msg, unknown_msg_tags, verbose = unparsed_tag_verbose)
                    if parse_error_verbose:
                        print('Unable to parse message: {}'.format(raw_msg))  
                except:
                    if parse_error_verbose:
                        print('Unable to parse or save message tag: {}'.format(raw_msg))
    
if log_stream:
    f = open(log_file_name, "a")

while True:
    raw_msg = s.recv(1024)  
    parse_nmea_message(raw_msg, loop_limit) 

    if log_stream:
        f.write(raw_msg)

    if time.time() > timeout and log_stream:
        f.close
        break