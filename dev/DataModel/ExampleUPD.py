from asyncio.windows_events import NULL
import socket
import time

seconds = 300
timeout = time.time() + seconds   

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("fagitrelay.it.ntnu.no",25508))

msg = NULL
sensor_data = {}
eol_string = '\r\n'
msg_types = {'!': 'type1_',
            '$': 'type2_' }

def parse_message(msg, msg_ref, eol_string): 
    msg_data = msg.strip(eol_string).split(',')
    msg_id = msg_data[0]
    #print(msg, msg_data[0])
    print(msg)
    for ref_key in msg_ref:
        msg_id = msg_id.replace(ref_key, msg_ref[ref_key])

    msg_data.pop(0)
    return msg_id, msg_data

def update_sensor_data(sensor_data, incoming_message):
    sensor_data[parsed_message[0]] = parsed_message[1]

f = open("datstream_5min.txt", "a")

while True:


    msg = s.recv(1024).decode()
    f.write(msg)
    parsed_message = parse_message(msg, msg_types, eol_string)
    update_sensor_data(sensor_data, parsed_message)

    if time.time() > timeout:
        f.close
        break
    #print(sensor_data) 