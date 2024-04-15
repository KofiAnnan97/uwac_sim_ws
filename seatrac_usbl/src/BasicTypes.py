import struct
import crcmod
from crcmod.predefined import *
import sys

# TODO: Correct encoding of settings

class BitOperations():
    def __init__(self):
        pass

    def get_crc16(self, in_str):
        outStr = '0000'
        tempStr = '000'
        crc16_func = crcmod.predefined.mkCrcFun('crc-16')
        if(sys.version[0]=='3'):
            return self.dectohexstr(self.swap_ui16(crc16_func(bytes.fromhex(in_str))),4)
        else:
            return self.dectohexstr(self.swap_ui16(crc16_func(in_str.decode('hex'))),4)

    def get_crc32(self, in_str):
        outStr  = '00000000'
        tempStr = '0000000'
        crc32_func = crcmod.predefined.mkCrcFun('crc-32')
        return self.dectohexstr(self.swap_ui32(crc32_func(bytes.fromhex(in_str))),8)

    def dectohexstr(self,input,size):
        val = input
        if(input < 0):
            val = (input + (1 << 4*size)) % (1 << 4*size)
        return '{0:0{1}X}'.format(val,size)

    def declisttohexstr(self,inputlist,size):
        tmpstr = ''
        swap_dict = {4:self.swap_ui16, 8:self.swap_ui32, 16:self.swap_ui64}
        for input in inputlist:
            if (size != 2):
                input = swap_dict[size](input)
            tmpstr += self.dectohexstr(input,size)
        return tmpstr

    def floatlisttohexstr(self,inputlist):
        tmpstr = ''
        for input in inputlist:
            tmpstr += self.floattohexstr(input)
        return tmpstr

    def hexstrtoasciistring(self,hexstr):
        tmplist = ''
        i = 0
        size = 2
        while (i < len(hexstr)):
            if(sys.version[0]=='3'):
                output = bytes.fromhex(hexstr[i:i+size]).decode('utf-8')
            else:
                output = hexstr[i:i+size].decode('hex')
            tmplist = tmplist + output
            i = i + size
        return tmplist

    def hexstrtodeclist(self,hexstr,size):
        tmplist = []
        i = 0
        swap_dict = {4:self.swap_ui16, 8:self.swap_ui32, 16:self.swap_ui64}
        while (i < len(hexstr)):
            if (size != 2):
                output = swap_dict[size](self.hexstrtodec(hexstr[i:i+size]))
            else:
                output = self.hexstrtodec(hexstr[i:i+size])
            tmplist.append(output)
            i = i + size
        return tmplist

    def hexstrtofloatlist(self,hexstr):
        tmplist = []
        i = 0
        size = 8
        while (i < len(hexstr)):
            tmplist.append(self.hexstrtofloat(hexstr[i:i+size]))
            i = i + size
        return tmplist

    def hexstrtodec(self,input):
        return int(input,16)

    def hexstrtosigndec(self,input):
        size = len(input)
        val = int(input,16)
        if((val & (1 << ((4*size)-1)))):
            val = val - (1 << 4*size)
        return val

    def strtohexascii(self,instr):
        tmpstr = ''
        for c in instr:
            tmpstr += format(ord(c), 'x')
        return tmpstr.upper()

    def floattohexstr(self,input):
        #return struct.pack('f', input).hex().upper()
        return struct.pack('f', input).encode('hex').upper()

    def hexstrtofloat(self,input):
        if(sys.version[0]=='3'):
            return struct.unpack('f', bytes.fromhex(input))[0]
        else:
            return struct.unpack('f',input.decode('hex'))[0]

    def swap_ui64(self,input):
        return struct.unpack("<Q", struct.pack(">Q", input))[0]

    def swap_i64(self,input):
        val = (input + (1 << 64)) % (1 << 64)
        return self.swap_ui64(val)

    def swap_ui32(self,input):
        return struct.unpack("<I", struct.pack(">I", input))[0]

    def swap_i32(self,input):
        val = (input + (1 << 32)) % (1 << 32)
        return self.swap_ui32(val)

    def swap_ui16(self,input):
        return struct.unpack("<H", struct.pack(">H", input))[0]

    def swap_i16(self,input):
        val = (input + (1 << 16)) % (1 << 16)
        return self.swap_ui16(val)

    def swap_float32(self,input):
        return struct.unpack("<f", struct.pack(">f", input))[0]

    def swap_float64(self,input):
        return struct.unpack("<d", struct.pack(">d", input))[0]

class CID_E:
    def __init__(self,cid = 'CID_DEFAULT'):
        self.Cid = cid
        self.DictVal = {'CID_DEFAULT'               : int('0xFF', 16),
                        'CID_SYS_ALIVE'             : int('0x01', 16),
                        'CID_SYS_INFO'              : int('0x02', 16),
                        'CID_SYS_REBOOT'            : int('0x03', 16),
                        'CID_SYS_ENGINEERING'       : int('0x04', 16),
                        'CID_PROG_INIT'             : int('0x0D', 16),
                        'CID_PROG_BLOCK'            : int('0x0E', 16),
                        'CID_PROG_UPDATE'           : int('0x0F', 16),
                        'CID_STATUS'                : int('0x10', 16),
                        'CID_STATUS_CFG_GET'        : int('0x11', 16),
                        'CID_STATUS_CFG_SET'        : int('0x12', 16),
                        'CID_SETTINGS_GET'          : int('0x15', 16),
                        'CID_SETTINGS_SET'          : int('0x16', 16),
                        'CID_SETTINGS_LOAD'         : int('0x17', 16),
                        'CID_SETTINGS_SAVE'         : int('0x18', 16),
                        'CID_SETTINGS_RESET'        : int('0x19', 16),
                        'CID_CAL_ACTION'            : int('0x20', 16),
                        'CID_AHRS_CAL_GET'          : int('0x21', 16),
                        'CID_AHRS_CAL_SET'          : int('0x22', 16),
                        'CID_XCVR_ANALYSE'          : int('0x30', 16),
                        'CID_XCVR_TX_MSG'           : int('0x31', 16),
                        'CID_XCVR_RX_ERR'           : int('0x32', 16),
                        'CID_XCVR_RX_MSG'           : int('0x33', 16),
                        'CID_XCVR_RX_REQ'           : int('0x34', 16),
                        'CID_XCVR_RX_RESP'          : int('0x35', 16),
                        'CID_XCVR_RX_UNHANDLED'     : int('0x37', 16),
                        'CID_XCVR_USBL'             : int('0x38', 16),
                        'CID_XCVR_FIX'              : int('0x39', 16),
                        'CID_XCVR_STATUS'           : int('0x3A', 16),
                        'CID_PING_SEND'             : int('0x40', 16),
                        'CID_PING_REQ'              : int('0x41', 16),
                        'CID_PING_RESP'             : int('0x42', 16),
                        'CID_PING_ERROR'            : int('0x43', 16),
                        'CID_ECHO_SEND'             : int('0x48', 16),
                        'CID_ECHO_REQ'              : int('0x49', 16),
                        'CID_ECHO_RESP'             : int('0x4A', 16),
                        'CID_ECHO_ERROR'            : int('0x4B', 16),
                        'CID_NAV_QUERY_SEND'        : int('0x50', 16),
                        'CID_NAV_QUERY_REQ'         : int('0x51', 16),
                        'CID_NAV_QUERY_RESP'        : int('0x52', 16),
                        'CID_NAV_ERROR'             : int('0x53', 16),
                        'CID_NAV_REF_POS_SEND'      : int('0x54', 16),
                        'CID_NAV_REF_POS_UPDATE'    : int('0x55', 16),
                        'CID_NAV_BEACON_POS_SEND'   : int('0x56', 16),
                        'CID_NAV_BEACON_POS_UPDATE' : int('0x57', 16),
                        'CID_NAV_QUEUE_SET'         : int('0x58', 16),
                        'CID_NAV_QUEUE_CLR'         : int('0x59', 16),
                        'CID_NAV_QUEUE_STATUS'      : int('0x5A', 16),
                        'CID_NAV_STATUS_SEND'       : int('0x5B', 16),
                        'CID_NAV_STATUS_RECEIVE'    : int('0x5C', 16),
                        'CID_DAT_SEND'              : int('0x60', 16),
                        'CID_DAT_RECEIVE'           : int('0x61', 16),
                        'CID_DAT_ERROR'             : int('0x63', 16),
                        'CID_DAT_QUEUE_SET'         : int('0x64', 16),
                        'CID_DAT_QUEUE_CLR'         : int('0x65', 16),
                        'CID_DAT_QUEUE_STATUS'      : int('0x66', 16),
                        'CID_DEX_CLOSE'             : int('0x70', 16),
                        'CID_DEX_DEBUG'             : int('0x71', 16),
                        'CID_DEX_ENQUEUE'           : int('0x72', 16),
                        'CID_DEX_OPEN'              : int('0x73', 16),
                        'CID_DEX_RESET'             : int('0x74', 16),
                        'CID_DEX_SEND'              : int('0x75', 16),
                        'CID_DEX_SOCKETS'           : int('0x76', 16),
                        'CID_DEX_RECEIVE'           : int('0x77', 16)}
        self.InvDictVal = dict(map(reversed, self.DictVal.items()))

    def encode_data_dec(self):
        return self.DictVal[self.Cid]

    def encode_data_hex(self):
        return BitOperations().dectohexstr(self.DictVal[self.Cid],2)

    def decode_data(self,rawdata):
        self.Cid = self.InvDictVal[BitOperations().hexstrtodec(rawdata)]
        return self.Cid

class BAUDRATE_E:
    def __init__(self,baud = 'B_115200'):
        self.Baud = baud
        self.DictVal = {'B_4800':7,'B_9600':8,'B_14400':9,'B_19200':10,'B_38400':11,'B_57600':12,'B_115200':13}
        self.InvDictVal = dict(map(reversed, self.DictVal.items()))

    def encode_data_dec(self):
        return self.DictVal[self.Baud]

    def encode_data_hex(self):
        return BitOperations().dectohexstr(self.DictVal[self.Baud],2)

    def decode_data(self,rawdata):
        self.Baud = self.InvDictVal[BitOperations().hexstrtodec(rawdata)]
        return self.Baud

class STATUSMODE_E:
    def __init__(self,status_mode = 'STATUS_MODE_MANUAL'):
        self.status_mode = status_mode
        self.DictVal = {'STATUS_MODE_MANUAL':0,'STATUS_MODE_1HZ':1,'STATUS_MODE_2HZ5':2,'STATUS_MODE_5HZ':3,'STATUS_MODE_10HZ':4,'STATUS_MODE_25HZ':5}
        self.InvDictVal = dict(map(reversed, self.DictVal.items()))

    def encode_data_dec(self):
        return self.DictVal[self.status_mode]

    def encode_data_hex(self):
        return BitOperations().dectohexstr(self.DictVal[self.status_mode],2)

    def decode_data(self,rawdata):
        self.status_mode = self.InvDictVal[BitOperations().hexstrtodec(rawdata)]
        return self.status_mode

class BID_E:
    def __init__(self,bid = 'BEACON_ID1'):
        self.Bid = bid
        self.DictVal = {'BEACON_DEFAULT':16,'BEACON_ALL':0,'BEACON_ID1':1,'BEACON_ID2':2,'BEACON_ID3':3,
                        'BEACON_ID4':4,'BEACON_ID5':5,'BEACON_ID6':6,'BEACON_ID7':7,
                        'BEACON_ID8':8,'BEACON_ID9':9,'BEACON_ID10':10,'BEACON_ID11':11,
                        'BEACON_ID12':12,'BEACON_ID13':13,'BEACON_ID14':14,'BEACON_ID15':15}
        self.InvDictVal = dict(map(reversed, self.DictVal.items()))

    def encode_data_dec(self):
        return self.DictVal[self.Bid]

    def encode_data_hex(self):
        return BitOperations().dectohexstr(self.DictVal[self.Bid],2)

    def decode_data(self,rawdata):
        self.Bid = self.InvDictVal[BitOperations().hexstrtodec(rawdata)]
        return self.Bid

class AMSGTYPE_E:
    def __init__(self,msgtype = 'MSG_UNKNOWN'):
        self.msgtype = msgtype
        self.DictVal = {'MSG_OWAY':0,'MSG_OWAYU':1,'MSG_REQ':2,'MSG_RESP':3,
                        'MSG_REQU':4,'MSG_RESPU':5,'MSG_REQX':6,'MSG_RESPX':7,
                        'MSG_UNKNOWN':255}
        self.InvDictVal = dict(map(reversed, self.DictVal.items()))

    def encode_data_dec(self):
        return self.DictVal[self.msgtype]

    def encode_data_hex(self):
        return BitOperations().dectohexstr(self.DictVal[self.msgtype],2)

    def decode_data(self,rawdata):
        self.msgtype = self.InvDictVal[BitOperations().hexstrtodec(rawdata)]
        return self.msgtype

class CST_E:
    def __init__(self,cmd_stat_code = 'CST_DEFAULT'):
        self.Cst = cmd_stat_code
        self.DictVal = {'CST_DEFAULT':                int('0xFF',16),
                        'CST_OK':                     int('0x00',16),
                        'CST_FAIL':                   int('0x01',16),
                        'CST_EEPROM_ERROR':           int('0x03',16),
                        'CST_CMD_PARAM_MISSING':      int('0x04',16),
                        'CST_CMD_PARAM_INVALID':      int('0x05',16),
                        'CST_PROG_FLASH_ERROR':       int('0x0A',16),
                        'CST_PROG_FIRMWARE_ERROR':    int('0x0B',16),
                        'CST_PROG_SECTION_ERROR':     int('0x0C',16),
                        'CST_PROG_LENGTH_ERROR':      int('0x0D',16),
                        'CST_PROG_DATA_ERROR':        int('0x0E',16),
                        'CST_PROG_CHECKSUM_ERROR':    int('0x0F',16),
                        'CST_XCVR_BUSY':              int('0x30',16),
                        'CST_XCVR_ID_REJECTED':       int('0x31',16),
                        'CST_XCVR_CSUM_ERROR':        int('0x32',16),
                        'CST_XCVR_LENGTH_ERROR':      int('0x33',16),
                        'CST_XCVR_RESP_TIMEOUT':      int('0x34',16),
                        'CST_XCVR_RESP_ERROR':        int('0x35',16),
                        'CST_XCVR_RESP_WRONG':        int('0x36',16),
                        'CST_XCVR_PLOAD_ERROR':       int('0x37',16),
                        'CST_XCVR_STATE_STOPPED':     int('0x3A',16),
                        'CST_XCVR_STATE_IDLE':        int('0x3B',16),
                        'CST_XCVR_STATE_TX':          int('0x3C',16),
                        'CST_XCVR_STATE_REQ':         int('0x3D',16),
                        'CST_XCVR_STATE_RX':          int('0x3E',16),
                        'CST_XCVR_STATE_RESP':        int('0x3F',16),
                        'CST_DEX_SOCKET_ERROR':       int('0x70',16),
                        'CST_DEX_RX_SYNC':            int('0x71',16),
                        'CST_DEX_RX_DATA':            int('0x72',16),
                        'CST_DEX_RX_SEQ_ERROR':       int('0x73',16),
                        'CST_DEX_RX_MSG_ERROR':       int('0x74',16),
                        'CST_DEX_REQ_ERROR':          int('0x75',16),
                        'CST_DEX_RESP_TMO_ERROR':     int('0x76',16),
                        'CST_DEX_RESP_MSG_ERROR':     int('0x77',16),
                        'CST_DEX_RESP_REMOTE_ERROR':  int('0x78',16)}
        self.InvDictVal = dict(map(reversed, self.DictVal.items()))

    def encode_data_dec(self):
        return self.DictVal[self.Cst]

    def encode_data_hex(self):
        return BitOperations().dectohexstr(self.DictVal[self.Cst],2)

    def decode_data(self,rawdata):
        self.Cst = self.InvDictVal[BitOperations().hexstrtodec(rawdata)]
        return self.Cst

class APAYLOAD_E:
    def __init__(self,plod_id = 'PLOAD_DEFAULT'):
        self.PloadId = plod_id
        self.DictVal = {'PLOAD_PING':0,'PLOAD_ECHO':1,'PLOAD_NAV':2,'PLOAD_DAT':3,'PLOAD_DEX':4,'PLOAD_DEFAULT':255}
        self.InvDictVal = dict(map(reversed, self.DictVal.items()))

    def encode_data_dec(self):
        return self.DictVal[self.PloadId]

    def encode_data_hex(self):
        return BitOperations().dectohexstr(self.DictVal[self.PloadId],2)

    def decode_data(self,rawdata):
        self.PloadId = self.InvDictVal[BitOperations().hexstrtodec(rawdata)]
        return self.PloadId

class IPADDR_T:
    def __init__(self,ip_arr = [192, 168, 1, 0]):
        self.ip_0 = ip_arr[0]
        self.ip_1 = ip_arr[1]
        self.ip_2 = ip_arr[2]
        self.ip_3 = ip_arr[3]

    def encode_data_hex(self):
        bo = BitOperations()
        return bo.dectohexstr(bo.swap_ui32(int(bo.dectohexstr(self.ip_0,2) + bo.dectohexstr(self.ip_1,2) + bo.dectohexstr(self.ip_2,2) + bo.dectohexstr(self.ip_3,2),16)),8)

    def encode_data_dec(self):
        bo = BitOperations()
        return int(self.encode_data_hex(),16)

    def decode_data(self,rawdata):
        self.ip_3 = int(rawdata[0:2],16)
        self.ip_2 = int(rawdata[2:4],16)
        self.ip_1 = int(rawdata[4:6],16)
        self.ip_0 = int(rawdata[6:8],16)
        return [self.ip_0, self.ip_1, self.ip_2, self.ip_3]

class MACADDR_T:
    def __init__(self,mac_arr = [0, 0, 0, 0, 0, 0]):
        self.mac_0 = mac_arr[0]
        self.mac_1 = mac_arr[1]
        self.mac_2 = mac_arr[2]
        self.mac_3 = mac_arr[3]
        self.mac_4 = mac_arr[4]
        self.mac_5 = mac_arr[5]

    def encode_data_hex(self):
        bo = BitOperations()
        return bo.dectohexstr(bo.swap_ui64(int(bo.dectohexstr(self.mac_0,2) + bo.dectohexstr(self.mac_1,2) + bo.dectohexstr(self.mac_2,2) + bo.dectohexstr(self.mac_3,2) + bo.dectohexstr(self.mac_4,2) + bo.dectohexstr(self.mac_5,2),16)),16)[0:12]

    def encode_data_dec(self):
        bo = BitOperations()
        return int(self.encode_data_hex(),16)

    def decode_data(self,rawdata):
        self.mac_5 = int(rawdata[0:2],16)
        self.mac_4 = int(rawdata[2:4],16)
        self.mac_3 = int(rawdata[4:6],16)
        self.mac_2 = int(rawdata[6:8],16)
        self.mac_1 = int(rawdata[8:10],16)
        self.mac_0 = int(rawdata[10:12],16)
        return [self.mac_0, self.mac_1, self.mac_2, self.mac_3, self.mac_4, self.mac_5]

class BITFIELD(object):
    def __init__(self, bitfield = [0, 0, 0, 0, 0, 0, 0, 0]):
        if len(bitfield) < 8:
            raise NameError('BITFIELD: Missing fields')
        self.b7 = bitfield[0]
        self.b6 = bitfield[1]
        self.b5 = bitfield[2]
        self.b4 = bitfield[3]
        self.b3 = bitfield[4]
        self.b2 = bitfield[5]
        self.b1 = bitfield[6]
        self.b0 = bitfield[7]
        # byte
        self.byte = (self.b7<<7) + (self.b6<<6) + (self.b5<<5) + (self.b4<<4) + (self.b3<<3) + (self.b2<<2) + (self.b1<<1) + (self.b0)

    def update_byte(self):
        # byte
        self.byte = (self.b7<<7) + (self.b6<<6) + (self.b5<<5) + (self.b4<<4) + (self.b3<<3) + (self.b2<<2) + (self.b1<<1) + (self.b0)

    def encode_data_hex(self):
        bo = BitOperations()
        return bo.dectohexstr(self.encode_data_dec(),2)

    def encode_data_dec(self):
        self.update_byte()
        return self.byte

    def decode_data(self,rawdata):
        self.b7 = ((int(rawdata[0:2],16) & (1<<7))>>7)
        self.b6 = ((int(rawdata[0:2],16) & (1<<8))>>6)
        self.b5 = ((int(rawdata[0:2],16) & (1<<5))>>5)
        self.b4 = ((int(rawdata[0:2],16) & (1<<4))>>4)
        self.b3 = ((int(rawdata[0:2],16) & (1<<3))>>3)
        self.b2 = ((int(rawdata[0:2],16) & (1<<2))>>2)
        self.b1 = ((int(rawdata[0:2],16) & (1<<1))>>1)
        self.b0 = int(rawdata[0:2],16) & (1)
        return [self.b7, self.b6, self.b5, self.b4, self.b3, self.b2, self.b1, self.b0]

class STATUS_BITS_T(BITFIELD):
    def __init__(self, bitfield = [0, 0, 0, 1, 1, 1, 1, 1]):
        if len(bitfield) < 8:
            raise NameError('STATUS_BITS_T: Missing fields')
        #super().__init__(bitfield)
        super(STATUS_BITS_T,self).__init__(bitfield)
        self.reserved0 = self.b7
        self.reserved1 = self.b6
        self.ahrs_comp_data = self.b5
        self.ahrs_raw_data = self.b4
        self.acc_cal = self.b3
        self.mag_cal = self.b2
        self.attitude = self.b1
        self.environment = self.b0
    # encode_data_xxx is implemented in the parent class. Decoding method is
    # overwritten to have the detailed information of each bit in the bifield.
    def decode_data(self,rawdata):
        tmplist = BITFIELD.decode_data(self,rawdata)
        self.ahrs_comp_data = self.b5
        self.ahrs_raw_data = self.b4
        self.acc_cal = self.b3
        self.mag_cal = self.b2
        self.attitude = self.b1
        self.environment = self.b0
        # byte
        self.update_byte()
        return tmplist

class ENV_FLAGS_T(BITFIELD):
    def __init__(self, bitfield = [0, 0, 0, 0, 0, 0, 1, 1]):
        if len(bitfield) < 8:
            raise NameError('STATUS_BITS_T: Missing fields')
        #super().__init__(bitfield)
        super(ENV_FLAGS_T,self).__init__(bitfield)
        self.reserved7 = self.b7
        self.reserved6 = self.b6
        self.reserved5 = self.b5
        self.reserved4 = self.b4
        self.reserved3 = self.b3
        self.reserved2 = self.b2
        self.auto_pressure_ofs = self.b1
        self.auto_vos = self.b0
    # encode_data_xxx is implemented in the parent class. Decoding method is
    # overwritten to have the detailed information of each bit in the bifield.
    def decode_data(self,rawdata):
        tmplist = BITFIELD.decode_data(self,rawdata)
        self.reserved7 = self.b7
        self.reserved6 = self.b6
        self.reserved5 = self.b5
        self.reserved4 = self.b4
        self.reserved3 = self.b3
        self.reserved2 = self.b2
        self.auto_pressure_ofs = self.b1
        self.auto_vos = self.b0
        # byte
        self.update_byte()
        return tmplist

class AHRSCAL_T:
    def __init__(self,params = [-270,-270,-270, 270, 270, 270, 0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 100.0, 0, 0, 0]):
        self.acc_min_x     = params[0]
        self.acc_min_y     = params[1]
        self.acc_min_z     = params[2]
        self.acc_max_x     = params[3]
        self.acc_max_y     = params[4]
        self.acc_max_z     = params[5]
        self.mag_valid     = params[6]
        self.mag_hard_x    = params[7]
        self.mag_hard_y    = params[8]
        self.mag_hard_z    = params[9]
        self.mag_soft_x    = params[10]
        self.mag_soft_y    = params[11]
        self.mag_soft_z    = params[12]
        self.mag_field     = params[13]
        self.mag_err       = params[14]
        self.gyro_ofs_x    = params[15]
        self.gyro_ofs_y    = params[16]
        self.gyro_ofs_z    = params[17]

    def encode_data_hex(self):
        bo = BitOperations()
        tmpstr = ''
        tmpstr += bo.dectohexstr(bo.swap_i16(self.acc_min_x),4) + bo.dectohexstr(bo.swap_i16(self.acc_min_y),4) + bo.dectohexstr(bo.swap_i16(self.acc_min_z),4)
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.acc_max_x),4) + bo.dectohexstr(bo.swap_ui16(self.acc_max_y),4) + bo.dectohexstr(bo.swap_ui16(self.acc_max_z),4)
        tmpstr += bo.dectohexstr(self.mag_valid,2)
        tmpstr += bo.floattohexstr(self.mag_hard_x) + bo.floattohexstr(self.mag_hard_y) + bo.floattohexstr(self.mag_hard_z)
        tmpstr += bo.floattohexstr(self.mag_soft_x) + bo.floattohexstr(self.mag_soft_y) + bo.floattohexstr(self.mag_soft_z)
        tmpstr += bo.floattohexstr(self.mag_field)
        tmpstr += bo.floattohexstr(self.mag_err)
        tmpstr += bo.dectohexstr(self.gyro_ofs_x,4) + bo.dectohexstr(self.gyro_ofs_y,4) + bo.dectohexstr(self.gyro_ofs_z,4)
        return tmpstr

    def encode_data_dec(self):
        return 0

    def decode_data(self,rawdata):
        bo = BitOperations()
        self.acc_min_x     = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[4:8])),4))
        self.acc_min_y     = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[4:8])),4))
        self.acc_min_z     = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[8:12])),4))
        self.acc_max_x     = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[12:16])),4))
        self.acc_max_y     = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[16:20])),4))
        self.acc_max_z     = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[20:24])),4))
        self.mag_valid     = bo.hexstrtodec(rawdata[24:26])
        self.mag_hard_x    = bo.hexstrtofloat(rawdata[26:34])
        self.mag_hard_y    = bo.hexstrtofloat(rawdata[34:42])
        self.mag_hard_z    = bo.hexstrtofloat(rawdata[42:50])
        self.mag_soft_x    = bo.hexstrtofloat(rawdata[50:58])
        self.mag_soft_y    = bo.hexstrtofloat(rawdata[58:66])
        self.mag_soft_z    = bo.hexstrtofloat(rawdata[66:74])
        self.mag_field     = bo.hexstrtofloat(rawdata[74:82])
        self.mag_err       = bo.hexstrtofloat(rawdata[82:90])
        self.gyro_offset_x = bo.hexstrtodec(rawdata[90:94])
        self.gyro_offset_y = bo.hexstrtodec(rawdata[94:98])
        self.gyro_offset_z = bo.hexstrtodec(rawdata[98:102])
        return [self.acc_min_x, self.acc_min_y , self.acc_min_z , self.acc_max_x , self.acc_max_y , self.acc_max_z , self.mag_valid , self.mag_hard_x, self.mag_hard_y , self.mag_hard_z , self.mag_soft_x , self.mag_soft_y , self.mag_soft_z , self.mag_field, self.mag_err , self.gyro_offset_x , self.gyro_offset_y , self.gyro_offset_z]

class FIRMWARE_T:
    def __init__(self,params = [255, 913, 2, 2, 2191, 0, 0]):
        self.valid         = params[0]
        self.part_number   = params[1]
        self.version_maj   = params[2]
        self.version_min   = params[3]
        self.version_build = params[4]
        self.checksum      = params[5]

    def encode_data_hex(self):
        bo = BitOperations()
        tmpstr = ''
        tmpstr += bo.dectohexstr(self.valid,2)
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.part_number),4)
        tmpstr += bo.dectohexstr(self.version_maj,2) + bo.dectohexstr(self.version_min,2) + bo.dectohexstr(bo.swap_ui16(self.version_build),4)
        tmpstr += bo.get_crc32(tmpstr)
        return tmpstr

    def encode_data_dec(self):
        return 0

    def decode_data(self,rawdata):
        bo = BitOperations()
        self.valid         = bo.hexstrtodec(rawdata[0:2])
        self.part_number   = bo.hexstrtodec(rawdata[2:6])
        self.version_maj   = bo.hexstrtodec(rawdata[6:8])
        self.version_min   = bo.hexstrtodec(rawdata[8:10])
        self.version_build = bo.hexstrtodec(rawdata[10:14])
        self.checksum      = bo.hexstrtodec(rawdata[14:22])
        return [self.valid, self.part_number, self.version_maj, self.version_min, self.version_build, self.checksum]

class HARDWARE_T:
    def __init__(self,params = [795, 6, 13014, 0, 0]):
        # 795 = SeaTrac X150 USBL Beacon
        # 843 = SeaTrac X110 Modem Beacon
        self.part_number   = params[0]
        self.part_rev      = params[1]
        self.serial_number = params[2]
        self.flags_sys     = params[3]
        self.flags_user    = params[4]


    def encode_data_hex(self):
        bo = BitOperations()
        tmpstr = ''
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.part_number),4)
        tmpstr += bo.dectohexstr(self.part_rev,2)
        tmpstr += bo.dectohexstr(bo.swap_ui32(self.serial_number),8)
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.flags_sys),4)
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.flags_user),4)
        return tmpstr

    def encode_data_dec(self):
        return 0

    def decode_data(self, rawdata):
        bo = BitOperations()
        self.part_number   = bo.swap_ui16(bo.hexstrtodec(rawdata[0:4]))
        self.part_rev      = bo.hexstrtodec(rawdata[4:6])
        self.serial_number = bo.swap_ui32(bo.hexstrtodec(rawdata[6:14]))
        self.flags_sys     = bo.swap_ui16(bo.hexstrtodec(rawdata[14:18]))
        self.flags_user    = bo.swap_ui16(bo.hexstrtodec(rawdata[18:22]))
        return [self.part_number, self.part_rev, self.serial_number, self.flags_sys, self.flags_user]

class NAV_QUERY_T(BITFIELD):
    def __init__(self, bitfield = [1, 1, 1, 1, 1, 1, 1, 1]):
        if len(bitfield) < 8:
            raise NameError('NAV_QUERY_T: Missing fields')
        #super().__init__(bitfield)
        super(NAV_QUERY_T,self).__init__(bitfield)
        self.qry_data     = self.b7
        self.reserved1    = self.b6
        self.reserved2    = self.b5
        self.reserved3    = self.b4
        self.qry_attitude = self.b3
        self.qry_temp     = self.b2
        self.qry_supply   = self.b1
        self.qry_depth    = self.b0
    # encode_data_xxx is implemented in the parent class. Decoding method is
    # overwritten to have the detailed information of each bit in the bifield.
    def decode_data(self,rawdata):
        tmplist = BITFIELD.decode_data(self,rawdata)
        self.qry_data    = self.b7
        self.reserved1    = self.b6
        self.reserved2    = self.b5
        self.reserved3    = self.b4
        self.qry_attitude = self.b3
        self.qry_temp     = self.b2
        self.qry_supply   = self.b1
        self.qry_depth    = self.b0
        # byte
        self.update_byte()
        return tmplist

class AHRS_FLAGS_T(BITFIELD):
    def __init__(self, bitfield = [0, 0, 0, 0, 0, 0, 0, 0]):
        if len(bitfield) < 8:
            raise NameError('AHRS_FLAGS_T: Missing fields')
        #super().__init__(bitfield)
        super(AHRS_FLAGS_T,self).__init__(bitfield)
        self.reserved0     = self.b7
        self.reserved1     = self.b6
        self.reserved2     = self.b5
        self.reserved3     = self.b4
        self.reserved4     = self.b3
        self.reserved5     = self.b2
        self.reserved6     = self.b1
        self.auto_cal_mag  = self.b0
    # encode_data_xxx is implemented in the parent class. Decoding method is
    # overwritten to have the detailed information of each bit in the bifield.
    def decode_data(self,rawdata):
        tmplist = BITFIELD.decode_data(self,rawdata)
        self.reserved0     = self.b7
        self.reserved1     = self.b6
        self.reserved2     = self.b5
        self.reserved3     = self.b4
        self.reserved4     = self.b3
        self.reserved5     = self.b2
        self.reserved6     = self.b1
        self.auto_cal_mag  = self.b0
        return tmplist

class XCVR_FLAGS_T(BITFIELD):
    def __init__(self, bitfield = [0, 0, 0, 0, 0, 0, 0, 0]):
        if len(bitfield) < 8:
            raise NameError('XCVR_FLAGS_T: Missing fields')
        #super().__init__(bitfield)
        super(XCVR_FLAGS_T,self).__init__(bitfield)
        self.xcvr_diag_msgs      = self.b7
        self.xcvr_fix_msgs       = self.b6
        self.xcvr_usbl_msgs      = self.b5
        self.reserved0           = self.b4
        self.reserved1           = self.b3
        self.reserved2           = self.b2
        self.xcvr_postflt_enable = self.b1
        self.usbl_use_ahrs       = self.b0
    # encode_data_xxx is implemented in the parent class. Decoding method is
    # overwritten to have the detailed information of each bit in the bifield.
    def decode_data(self, rawdata):
        tmplist = BITFIELD.decode_data(self,rawdata)
        self.xcvr_diag_msgs      = self.b7
        self.xcvr_fix_msgs       = self.b6
        self.xcvr_usbl_msgs      = self.b5
        self.reserved0           = self.b4
        self.reserved1           = self.b3
        self.reserved2           = self.b2
        self.xcvr_postflt_enable = self.b1
        self.usbl_use_ahrs       = self.b0
        # byte
        self.update_byte()
        return tmplist

class ACOFIX_FLAGS_T(BITFIELD):
    def __init__(self, bitfield = [0, 0, 0, 0, 0, 0, 0, 0]):
        if len(bitfield) < 8:
            raise NameError('XCVR_FLAGS_T: Missing fields')
        #super().__init__(bitfield)
        super(ACOFIX_FLAGS_T,self).__init__(bitfield)
        self.reserved0          = self.b7
        self.reserved1          = self.b6
        self.reserved2          = self.b5
        self.position_flt_error = self.b4
        self.position_enhanced  = self.b3
        self.position_valid     = self.b2
        self.usbl_valid         = self.b1
        self.range_valid        = self.b0

    # encode_data_xxx is implemented in the parent class. Decoding method is
    # overwritten to have the detailed information of each bit in the bifield.
    def decode_data(self, rawdata):
        tmplist = BITFIELD.decode_data(self,rawdata)
        self.reserved0          = self.b7
        self.reserved1          = self.b6
        self.reserved2          = self.b5
        self.position_flt_error = self.b4
        self.position_enhanced  = self.b3
        self.position_valid     = self.b2
        self.usbl_valid         = self.b1
        self.range_valid        = self.b0
        # byte
        self.update_byte()
        return tmplist

class SETTINGS_T:
    def __init__(self):
        self.status_flags_e = STATUSMODE_E('STATUS_MODE_MANUAL')
        self.status_output_t = STATUS_BITS_T([0, 0, 0, 1, 1, 1, 1, 1])
        self.uart_main_baud_e = BAUDRATE_E('B_115200')
        self.uart_aux_baud_e = BAUDRATE_E('B_115200')
        self.net_mac_addr_t = MACADDR_T([0,0,0,0,0,0])
        self.net_ip_addr_t = IPADDR_T([192,168,1,250])
        self.net_ip_subnet_t = IPADDR_T([255,255,0,0])
        self.net_ip_gateway_t = IPADDR_T([192,168,1,1])
        self.net_ip_dns_t = IPADDR_T([192,168,1,1])
        self.net_tcp_port_u16 = 8100
        self.env_flags_t = ENV_FLAGS_T([0,0,0,0,0,0,1,1])
        self.env_pres_ofs_i32 = 5
        self.env_salinity_u16 = 35
        self.env_vos_u16 = 15160
        self.arhs_flags_t = AHRS_FLAGS_T([0,0,0,0,0,0,0,0])
        self.arhs_cal_t = AHRSCAL_T([-270,-270,-270, 270, 270, 270, 0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 100.0, 0, 0, 0])
        self.ahrs_yaw_ofs_u16 = 0
        self.ahrs_pitch_ofs_u16 = 0
        self.ahrs_roll_ofs_u16 = 0
        self.xcvr_flags_t = XCVR_FLAGS_T([0,1,1,0,0,0,1,1])
        self.xcvr_beacon_id_e = BID_E('BEACON_ID15')
        self.xcvr_range_tmo_u16 = 1000
        self.xcvr_resp_time_u16 = 10
        self.xcvr_yaw_u16 = 0
        self.xcvr_pitch_u16 = 0
        self.xcvr_roll_u16 = 0
        self.xcvr_posflt_vel_u8 = 3
        self.xcvr_posflt_ang_u8 = 10
        self.xcvr_posflt_tmo_u8 = 60

    def encode_data_hex(self):
        bo = BitOperations()
        tmpstr = ''
        tmpstr += self.status_flags_e.encode_data_hex()
        tmpstr += self.status_output_t.encode_data_hex()
        tmpstr += self.uart_main_baud_e.encode_data_hex()
        tmpstr += self.uart_aux_baud_e.encode_data_hex()
        tmpstr += self.net_mac_addr_t.encode_data_hex()
        tmpstr += self.net_ip_addr_t.encode_data_hex()
        tmpstr += self.net_ip_subnet_t.encode_data_hex()
        tmpstr += self.net_ip_gateway_t.encode_data_hex()
        tmpstr += self.net_ip_dns_t.encode_data_hex()
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.net_tcp_port_u16),4)
        tmpstr += self.env_flags_t.encode_data_hex()
        tmpstr += bo.dectohexstr(bo.swap_i32(self.env_pres_ofs_i32),8)
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.env_salinity_u16*10),4)
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.env_vos_u16),4)
        tmpstr += self.arhs_flags_t.encode_data_hex()
        tmpstr += self.arhs_cal_t.encode_data_hex()
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.ahrs_yaw_ofs_u16),4)
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.ahrs_pitch_ofs_u16),4)
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.ahrs_roll_ofs_u16),4)
        tmpstr += self.xcvr_flags_t.encode_data_hex()
        tmpstr += self.xcvr_beacon_id_e.encode_data_hex()
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.xcvr_range_tmo_u16),4)
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.xcvr_resp_time_u16),4)
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.xcvr_yaw_u16),4)
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.xcvr_pitch_u16),4)
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.xcvr_roll_u16),4)
        tmpstr += bo.dectohexstr(self.xcvr_posflt_vel_u8,2)
        tmpstr += bo.dectohexstr(self.xcvr_posflt_ang_u8,2)
        tmpstr += bo.dectohexstr(self.xcvr_posflt_tmo_u8,2)
        return tmpstr

    def encode_data_dec(self):
        return 0

    def decode_data(self,rawdata):
        bo = BitOperations()
        self.status_flags_e.decode_data(rawdata[0:2])
        self.status_output_t.decode_data(rawdata[2:4])
        self.uart_main_baud_e.decode_data(rawdata[4:6])
        self.uart_aux_baud_e.decode_data(rawdata[6:8])
        self.net_mac_addr_t.decode_data(rawdata[8:20])
        self.net_ip_addr_t.decode_data(rawdata[20:28])
        self.net_ip_subnet_t.decode_data(rawdata[28:36])
        self.net_ip_gateway_t.decode_data(rawdata[36:44])
        self.net_ip_dns_t.decode_data(rawdata[44:52])
        self.net_tcp_port_u16 = bo.swap_ui16(bo.hexstrtodec(rawdata[52:56]))
        self.env_flags_t.decode_data(rawdata[56:58])
        self.env_pres_ofs_i32 = bo.swap_i32(bo.hexstrtodec(rawdata[58:66]))
        self.env_salinity_u16 = bo.swap_ui16(bo.hexstrtodec(rawdata[66:70]))/10
        self.env_vos_u16 = bo.swap_ui16(bo.hexstrtodec(rawdata[70:74]))
        self.arhs_flags_t.decode_data(rawdata[74:76])
        self.arhs_cal_t.decode_data(rawdata[76:178])
        self.ahrs_yaw_ofs_u16 = bo.swap_ui16(bo.hexstrtodec(rawdata[178:182]))
        self.ahrs_pitch_ofs_u16 = bo.swap_ui16(bo.hexstrtodec(rawdata[182:186]))
        self.ahrs_roll_ofs_u16 = bo.swap_ui16(bo.hexstrtodec(rawdata[186:190]))
        self.xcvr_flags_t.decode_data(rawdata[190:192])
        self.xcvr_beacon_id_e.decode_data(rawdata[192:194])
        self.xcvr_range_tmo_u16 = bo.swap_ui16(bo.hexstrtodec(rawdata[194:198]))
        self.xcvr_resp_time_u16 = bo.swap_ui16(bo.hexstrtodec(rawdata[198:202]))
        self.xcvr_yaw_u16 = bo.swap_ui16(bo.hexstrtodec(rawdata[202:206]))
        self.xcvr_pitch_u16 = bo.swap_ui16(bo.hexstrtodec(rawdata[206:210]))
        self.xcvr_roll_u16 = bo.swap_ui16(bo.hexstrtodec(rawdata[210:214]))
        self.xcvr_posflt_vel_u8 = bo.hexstrtodec(rawdata[214:216])
        self.xcvr_posflt_ang_u8 = bo.hexstrtodec(rawdata[216:218])
        self.xcvr_posflt_tmo_u8 = bo.hexstrtodec(rawdata[218:220])
        return

class ACOFIX_T:
    def __init__(self):
        self.dest_id_e = BID_E('BEACON_DEFAULT')
        self.src_id_e = BID_E('BEACON_DEFAULT')
        self.acofix_flags_t = ACOFIX_FLAGS_T([0, 0, 0, 1, 1, 1, 1, 1])
        self.msg_type_e = AMSGTYPE_E('MSG_RESPU')
        self.attitude_yaw_i16 = 1678                   # deci-degrees
        self.attitude_pitch_i16 = -855                 # deci-degrees
        self.attitude_roll_i16 = 129                  # deci-degrees
        self.depth_local_u16 = 0                    # deci-meters
        self.vos_u16 = 15196                            # deci-meters/second
        self.rssi_i16 = 1344                           # divide this value by 10 to get decibels
        # range Fields
        self.range_count_u32 = 972                    # 16Khz clock ticks between transmission and reception of acknowledge
        self.range_time_i32 = 7499                     # time in 100 nanoseconds derived from range_count_u32 + internal timming and offsets
        self.range_dist_u16 = 5                     # Line of sight distance (in decimeters) based on RANGE_TIME and VOS values
        # usbl fields
        self.usbl_channels_u8 = 4                   # no of usbl channels used to compute the signal angle (3 or 4 typically)
        self.usbl_rssi_i16 = [1691,1720,1716,1723]  # Strenght for each of the usbl receiver channels in centibels
        self.usbl_azimuth_i16 = 1168                   # value in deci-degrees
        self.usbl_elevation_i16 = 424                 # value in deci-degrees
        self.usbl_fit_error_i16 = 168                 # error in azimuth and elevation calculation 0: no error, >2 or >3 poor fitting.
        # Positional fields
        self.position_easting_i16 = -3               # value in deci-meters
        self.position_northing_i16 = -3              # value in deci-meters
        self.position_depth_i16 = -1                 # value in deci-meters
        self.rcvd_length = 0

    def encode_data_hex(self):
        bo = BitOperations()
        tmpstr = ''
        tmpstr += self.dest_id_e.encode_data_hex()
        tmpstr += self.src_id_e.encode_data_hex()
        tmpstr += self.acofix_flags_t.encode_data_hex()
        tmpstr += self.msg_type_e.encode_data_hex()
        tmpstr += bo.dectohexstr(bo.swap_i16(self.attitude_yaw_i16),4)
        tmpstr += bo.dectohexstr(bo.swap_i16(self.attitude_pitch_i16),4)
        tmpstr += bo.dectohexstr(bo.swap_i16(self.attitude_roll_i16),4)
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.depth_local_u16),4)
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.vos_u16),4)
        tmpstr += bo.dectohexstr(bo.swap_i16(self.rssi_i16),4)
        tmpstr += bo.dectohexstr(bo.swap_ui32(self.range_count_u32),8)
        tmpstr += bo.dectohexstr(bo.swap_ui32(self.range_time_i32),8)
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.range_dist_u16),4)
        tmpstr += bo.dectohexstr(self.usbl_channels_u8,2)
        tmpstr += bo.declisttohexstr(self.usbl_rssi_i16,4)
        tmpstr += bo.dectohexstr(bo.swap_i16(self.usbl_azimuth_i16),4)
        tmpstr += bo.dectohexstr(bo.swap_i16(self.usbl_elevation_i16),4)
        tmpstr += bo.dectohexstr(bo.swap_i16(self.usbl_fit_error_i16),4)
        tmpstr += bo.dectohexstr(bo.swap_i16(self.position_easting_i16),4)
        tmpstr += bo.dectohexstr(bo.swap_i16(self.position_northing_i16),4)
        tmpstr += bo.dectohexstr(bo.swap_i16(self.position_depth_i16),4)
        return tmpstr

    def encode_data_dec(self):
        return 0

    def decode_data(self,rawdata):
        bo = BitOperations()
        tmplist = []
        # Decode the response
        st = 0; sz = 2;    self.dest_id_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.dest_id_e.Bid)
        st += sz; sz = 2;  self.src_id_e.decode_data(rawdata[st:st+sz]); tmplist.append(self.src_id_e.Bid)
        st += sz; sz = 2;  self.acofix_flags_t.decode_data(rawdata[st:st+sz]); tmplist.append(self.acofix_flags_t.byte)
        st += sz; sz = 2;  self.msg_type_e.decode_data(rawdata[st:st+sz]); tmplist.append(self.msg_type_e.msgtype)
        st += sz; sz = 4;  self.attitude_yaw_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.attitude_yaw_i16)
        st += sz; sz = 4;  self.attitude_pitch_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.attitude_pitch_i16)
        st += sz; sz = 4;  self.attitude_roll_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.attitude_roll_i16)
        st += sz; sz = 4;  self.depth_local_u16 = bo.hexstrtodec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.depth_local_u16)
        st += sz; sz = 4;  self.vos_u16 = bo.hexstrtodec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.vos_u16)
        st += sz; sz = 4;  self.rssi_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.rssi_i16)
        # Only decode if the frame contains range fields
        if self.acofix_flags_t.range_valid == 1:
            st += sz; sz = 8;  self.range_count_u32 = bo.hexstrtodec(bo.dectohexstr(bo.swap_ui32(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.range_count_u32)
            st += sz; sz = 8;  self.range_time_i32 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui32(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.range_time_i32)
            st += sz; sz = 4;  self.range_dist_u16 = bo.hexstrtodec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.range_dist_u16)
        # Only decode if the frame contains usbl fields
        if self.acofix_flags_t.usbl_valid == 1:
            st += sz; sz = 2;  self.usbl_channels_u8 = bo.hexstrtodec(rawdata[st:st+sz]); tmplist.append(self.usbl_channels_u8)
            st += sz; sz = 4*self.usbl_channels_u8;  self.usbl_rssi_i16 = bo.hexstrtodeclist(rawdata[st:st+sz],4); tmplist.append(self.usbl_rssi_i16)
            st += sz; sz = 4;  self.usbl_azimuth_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.usbl_azimuth_i16)
            st += sz; sz = 4;  self.usbl_elevation_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.usbl_elevation_i16)
            st += sz; sz = 4;  self.usbl_fit_error_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.usbl_fit_error_i16)
        # Only decode if the frame contains position fields
        if self.acofix_flags_t.position_valid == 1:
            st += sz; sz = 4;  self.position_easting_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.position_easting_i16)
            st += sz; sz = 4;  self.position_northing_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.position_northing_i16)
            st += sz; sz = 4;  self.position_depth_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.position_depth_i16)
        self.rcvd_length = st+sz
        return tmplist

class ACOMSG_T:
    def __init__(self):
        self.dest_id_e = BID_E('BEACON_DEFAULT')
        self.src_id_e = BID_E('BEACON_DEFAULT')
        self.msg_type_e = AMSGTYPE_E('MSG_RESPU')
        self.depth_u16 = 0
        self.msg_pload_id_e = APAYLOAD_E()
        self.msg_pload_len_u8 = 31
        self.msg_pload_au8 = [0]*31

    def encode_data_hex(self):
        bo = BitOperations()
        tmpstr = ''
        tmpstr += self.dest_id_e.encode_data_hex()
        tmpstr += self.src_id_e.encode_data_hex()
        tmpstr += self.msg_type_e.encode_data_hex()
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.depth_u16),4)
        tmpstr += self.msg_pload_id_e.encode_data_hex()
        tmpstr += bo.dectohexstr(self.msg_pload_len_u8,2)
        tmpstr += bo.declisttohexstr(self.msg_pload_au8,2)
        return tmpstr

    def encode_data_dec(self):
        return 0

    def decode_data(self,rawdata):
        bo = BitOperations()
        tmplist = []
        # Decode the response
        st = 0; sz = 2;    self.dest_id_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.dest_id_e.Bid)
        st += sz; sz = 2;  self.src_id_e.decode_data(rawdata[st:st+sz]); tmplist.append(self.src_id_e.Bid)
        st += sz; sz = 2;  self.msg_type_e.decode_data(rawdata[st:st+sz]); tmplist.append(self.msg_type_e.msgtype)
        st += sz; sz = 4;  self.depth_u16 = bo.hexstrtodec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.depth_u16)
        st += sz; sz = 2;  self.msg_pload_id_e.decode_data(rawdata[st:st+sz]); tmplist.append(self.msg_pload_id_e.PloadId)
        st += sz; sz = 2;  self.msg_pload_len_u8 = bo.hexstrtodec(rawdata[st:st+sz]); tmplist.append(self.msg_pload_len_u8)
        st += sz; sz = 2*self.msg_pload_len_u8;  self.msg_pload_au8 = bo.hexstrtodeclist(rawdata[st:st+sz],2); tmplist.append(self.msg_pload_au8)
        self.rcvd_length = st+sz
        return tmplist

if __name__ == '__main__':
    print('CID_E encode_hex          : %s'%(CID_E('CID_SETTINGS_SET').encode_data_hex()))
    print('CID_E encode_dec          : %d'%(CID_E('CID_SETTINGS_SET').encode_data_dec()))
    print('CID_E decode_data         : %s\n'%(CID_E().decode_data(CID_E('CID_SETTINGS_SET').encode_data_hex())))

    print('BAUDRATE_E encode_hex     : %s'%(BAUDRATE_E('B_115200').encode_data_hex()))
    print('BAUDRATE_E encode_dec     : %d'%(BAUDRATE_E('B_115200').encode_data_dec()))
    print('BAUDRATE_E decode_data    : %s\n'%(BAUDRATE_E().decode_data(BAUDRATE_E('B_115200').encode_data_hex())))

    print('IPADDR_T encode_hex       : %s'%(IPADDR_T([192, 168, 1, 250]).encode_data_hex()))
    print('IPADDR_T encode_dec       : %d'%(IPADDR_T([192, 168, 1, 250]).encode_data_dec()))
    print('IPADDR_T decode_data      : %s\n'%(IPADDR_T().decode_data(IPADDR_T([192, 168, 1, 250]).encode_data_hex())))

    print('MACADDR_T encode_hex      : %s'%(MACADDR_T([10, 11, 12, 13, 14, 15]).encode_data_hex()))
    print('MACADDR_T encode_dec      : %d'%(MACADDR_T([10, 11, 12, 13, 14, 15]).encode_data_dec()))
    print('MACADDR_T decode_data     : %s\n'%(MACADDR_T().decode_data(MACADDR_T([10, 11, 12, 13, 14, 15]).encode_data_hex())))

    print('STATUS_BITS_T encode_hex  : %s'%(STATUS_BITS_T([0, 0, 0, 1, 1, 1, 1, 1]).encode_data_hex()))
    print('STATUS_BITS_T encode_dec  : %d'%(STATUS_BITS_T([0, 0, 0, 1, 1, 1, 1, 1]).encode_data_dec()))
    print('STATUS_BITS_T decode_data : %s\n'%(STATUS_BITS_T().decode_data(STATUS_BITS_T([0, 0, 0, 1, 1, 1, 1, 1]).encode_data_hex())))

    print('ENV_FLAGS_T encode_hex    : %s'%(ENV_FLAGS_T([0, 0, 0, 0, 0, 0, 1, 1]).encode_data_hex()))
    print('ENV_FLAGS_T encode_dec    : %d'%(ENV_FLAGS_T([0, 0, 0, 0, 0, 0, 1, 1]).encode_data_dec()))
    print('ENV_FLAGS_T decode_data   : %s\n'%(ENV_FLAGS_T().decode_data(ENV_FLAGS_T([0, 0, 0, 0, 0, 0, 1, 1]).encode_data_hex())))

    print('AHRSCAL_T encode_hex      : %s'%(AHRSCAL_T([-270,-270,-270, 270, 270, 270, 0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 100.0, 0, 0, 0]).encode_data_hex()))
    #print('AHRSCAL_T encode_dec     : %d'%(AHRSCAL_T([-270,-270,-270, 270, 270, 270, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0, 0, 0]).encode_data_dec()))
    print('AHRSCAL_T decode_data     : %s\n'%(AHRSCAL_T().decode_data(AHRSCAL_T([-270,-270,-270, 270, 270, 270, 0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 100.0, 0, 0, 0]).encode_data_hex())))

    print('FIRMWARE_T encode_hex     : %s'%(FIRMWARE_T([255, 913, 2, 2, 2191, 0, 0]).encode_data_hex()))
    #print('FIRMWARE_T encode_dec    : %d'%(FIRMWARE_T([0, 0, 0, 0, 0, 0, 1, 1]).encode_data_dec()))
    print('FIRMWARE_T decode_data    : %s\n'%(FIRMWARE_T().decode_data(FIRMWARE_T([255, 913, 2, 2, 2191, 0, 0]).encode_data_hex())))

    print('HARDWARE_T encode_hex     : %s'%(HARDWARE_T([795, 6, 13014, 0, 0]).encode_data_hex()))
    #print('FIRMWARE_T encode_dec    : %d'%(FIRMWARE_T([0, 0, 0, 0, 0, 0, 1, 1]).encode_data_dec()))
    print('HARDWARE_T decode_data    : %s\n'%(HARDWARE_T().decode_data(HARDWARE_T([795, 6, 13014, 0, 0]).encode_data_hex())))

    print('NAV_QUERY_T encode_hex    : %s'%(NAV_QUERY_T([0, 0, 0, 0, 1, 1, 1, 1]).encode_data_hex()))
    print('NAV_QUERY_T encode_dec    : %d'%(NAV_QUERY_T([0, 0, 0, 0, 1, 1, 1, 1]).encode_data_dec()))
    print('NAV_QUERY_T decode_data   : %s\n'%(NAV_QUERY_T().decode_data(NAV_QUERY_T([0, 0, 0, 0, 1, 1, 1, 1]).encode_data_hex())))

    print('XCVR_FLAGS_T encode_hex    : %s'%(XCVR_FLAGS_T([0, 0, 0, 0, 1, 1, 1, 1]).encode_data_hex()))
    print('XCVR_FLAGS_T encode_dec    : %d'%(XCVR_FLAGS_T([0, 0, 0, 0, 1, 1, 1, 1]).encode_data_dec()))
    print('XCVR_FLAGS_T decode_data   : %s\n'%(XCVR_FLAGS_T().decode_data(XCVR_FLAGS_T([0, 0, 0, 0, 1, 1, 1, 1]).encode_data_hex())))

    print('SETTINGS_T encode_hex    : %s'%(SETTINGS_T().encode_data_hex()))
    #print('SETTINGS_T encode_dec    : %d'%(SETTINGS_T().encode_data_dec()))
    print('SETTINGS_T decode_data   : %s\n'%(SETTINGS_T().decode_data(SETTINGS_T().encode_data_hex())))

    print('ACOFIX_T encode_hex    : %s'%(ACOFIX_T().encode_data_hex()))
    #print('ACOFIX_T encode_dec    : %d'%(SETTINGS_T().encode_data_dec()))
    print('ACOFIX_T decode_data   : %s\n'%(ACOFIX_T().decode_data(ACOFIX_T().encode_data_hex())))
