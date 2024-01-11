import os
import binascii
from BasicTypes import *

class AppMsg_Text():
    """
    AppMsg_Text defines a message aimed to be populated with a single string
    """
    def __init__(self,d_text="Default"):
        self.id = 0     # This number should be unique for each message class
        self.d_text = d_text

    def encode_message(self):
        bo = BitOperations()
        tmp_str = ''
        tmp_str += bo.dectohexstr(self.id,2)
        tmp_str += bo.strtohexascii(self.d_text)
        # Just pass the input parameter directly, since no conversion is required
        return tmp_str
    
    def decode_message(self,msg):
        bo = BitOperations()
        st = 0; sz = 2
        id = bo.hexstrtodec(msg[st:st+sz])
        st += sz; sz = 0
        data = bo.hexstrtoasciistring(msg[st:])
        return id,data

class AppMsg_Gps():
    """
    AppMsg_Gps defines a message aimed to be populated with latitude and longitude data
    """
    def __init__(self,data=[0.0,0.0]):
        self.id = 1     # This number should be unique for each message class
        self.d_long, self.d_lat = data 
    
    def encode_message(self):
        bo = BitOperations()
        tmp_str = ''
        tmp_str += bo.dectohexstr(self.id,2)
        tmp_str += bo.floattohexstr(self.d_long)
        tmp_str += bo.floattohexstr(self.d_lat)
        return tmp_str
    
    def decode_message(self, msg):
        bo = BitOperations()
        data = []
        st = 0; sz = 2
        id = bo.hexstrtodec(msg[st:st+sz])
        st += sz; sz = 8
        data.append(bo.hexstrtofloat(msg[st:st+sz]))
        st += sz; sz = 8
        data.append(bo.hexstrtofloat(msg[st:]))
        return id, data 

class AppMsg_GpsHead():
    """
    AppMsg_Gps defines a message aimed to be populated with latitude and longitude data
    """
    def __init__(self,data=[0.0, 0.0, 0.0]):
        self.id = 2     # This number should be unique for each message class
        self.d_long, self.d_lat, self.d_head = data 
    
    def encode_message(self):
        bo = BitOperations()
        tmp_str = ''
        tmp_str += bo.dectohexstr(self.id,2)
        tmp_str += bo.floattohexstr(self.d_long)
        tmp_str += bo.floattohexstr(self.d_lat)
        tmp_str += bo.floattohexstr(self.d_head)
        return tmp_str
    
    def decode_message(self, msg):
        bo = BitOperations()
        data = []
        st = 0; sz = 2
        id = bo.hexstrtodec(msg[st:st+sz])
        st += sz; sz = 8
        data.append(bo.hexstrtofloat(msg[st:st+sz]))
        st += sz; sz = 8
        data.append(bo.hexstrtofloat(msg[st:st+sz]))
        st += sz; sz = 8
        data.append(bo.hexstrtofloat(msg[st:]))
        return id, data   

if __name__ == "__main__":
    # Example 1
    print(AppMsg_GpsHead([40.5,-15.2,15.2]).encode_message())
    print(AppMsg_GpsHead([40.5,-15.2,15.2]).decode_message(AppMsg_GpsHead([40.5,-15.2,15.2]).encode_message()))
    # Example 2
    obj = AppMsg_GpsHead()
    obj.d_lat = 40.324
    obj.d_long = -86.123
    obj.d_head = -12.687
    msg = obj.encode_message()
    print(msg)
    print(obj.decode_message(msg))
    # Example 3
    print(AppMsg_Gps([40.5,-15.2]).encode_message())
    print(AppMsg_Gps([40.5,-15.2]).decode_message(AppMsg_Gps([40.5,-15.2]).encode_message()))
    # Example 4
    obj = AppMsg_Gps()
    obj.d_lat = 40.324
    obj.d_long = -86.123
    msg = obj.encode_message()
    print(msg)
    print(obj.decode_message(msg))
    # Example 5
    print(AppMsg_Text("Test").encode_message())
    print(AppMsg_Text("Test").decode_message(AppMsg_Text("Test").encode_message()))
    # Example 6
    obj = AppMsg_Text()
    obj.d_text = "Test String"
    msg = obj.encode_message()
    print(msg)
    print(obj.decode_message(msg))
