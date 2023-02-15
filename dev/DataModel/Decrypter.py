import Crypto
from Crypto.PublicKey import RSA 
from base64 import b64decode  
import os 
import re

class Decrypter:
    def __init__(self, key_path, talker = '!U9SEC', field_num = 5, max_id = 10):
        self.talker = talker
        self.key_path = key_path
        self._buffer = [None] * max_id 
        self._keyring = {}
        self._field_num = field_num

    def _assemble(self, raw_msg):
        decoded = raw_msg.decode(encoding='ascii')  

        # check if receiving only one message
        assert(len(decoded.split(',')) == self._field_num)
        
        talker_id, msg_len, msg_seq, msg_id, content = decoded.split(',')
        msg_len = int(msg_len)
        msg_seq = int(msg_seq)-1
        msg_id = int(msg_id)

        # check if talker is correct
        assert(talker_id.count(self.talker)) 

        if self._buffer[msg_id] is None:
            self._buffer[msg_id] = [None] * msg_len
            
        self._buffer[msg_id][msg_seq] = content.strip()

        full_content = ''
        if None in self._buffer[msg_id]:
            return full_content
        else:
            full_content = full_content.join(self._buffer[msg_id])
            self._buffer[msg_id] = None
            return full_content 

    def _get_key_cypher(self, full_content):
        seq = full_content.split(';') 
        key_name, content = seq[0], seq[1:]
        
        key_name = key_name.strip() + '.key.pub' 
        # use regex
        key_name = key_name.replace('/','_').replace(' ','_')

        if len(content) > 1: content = ''.join(content)

        cypher = content[0].encode('ascii') 

        if key_name in self._keyring:
            key = self._keyring[key_name] 
        else: 
            key_path = os.path.join(self.key_path, key_name) 
            key_string = open(key_path, "r").read()
            
            key = RSA.importKey(key_string)
            self._keyring[key_name] = key
        
        return key, cypher
    
    def _clean_message(self, decrypted): 
        decrypted = decrypted.decode("ascii", 'backslashreplace')
        decrypted = re.sub(r"([\\][x][a-z0-9]{2})+", "", decrypted) 
        decrypted = decrypted[1:]
        seq = decrypted.split(';')
        metadata_str, msg = seq[0], seq[1:]
        if len(msg) > 1: msg = ''.join(msg) 
        unix_time, seq_num, src_id, src_name = metadata_str.split(',') 
        
        metadata = (
            float(unix_time[1:]), # bit is appended to string somehow  
            int(seq_num), 
            int(src_id), 
            src_name
        )

        if type(msg) is list: msg = msg[0]

        return metadata, msg 
        
    def decrypt(self, raw_msg): 
        
        full_content = self._assemble(raw_msg)  
        if full_content == '': return ''
        
        key, cypher = self._get_key_cypher(full_content)
        raw_cipher_data = b64decode(cypher) 
        decrypted = key.encrypt(raw_cipher_data, 0)
        clean = self._clean_message(decrypted[0])
        return clean
