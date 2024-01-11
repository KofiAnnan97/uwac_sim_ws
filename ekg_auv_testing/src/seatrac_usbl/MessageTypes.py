from BasicTypes import *
import matplotlib.pyplot as plt

class MsgOperation():

    def __init__(self):
        # Class variables
        self.START_CHAR_TX = '#'
        self.START_CHAR_RX = '$'
        self.TERMINATOR_STR = '\r\n'
        self.__MSG_TYPE_DICT = {self.START_CHAR_TX:'TX', self.START_CHAR_RX: 'RX'}

    def getMessageId(self, in_raw_message):
        start = in_raw_message.find(self.START_CHAR_RX)
        if(start == -1):
            start = 1
        else:
            start = start + 1
        cmdIdObj = CID_E('CID_DEFAULT')
        cmdIdObj.decode_data(in_raw_message[start:start+2])
        return cmdIdObj.Cid

    def getMsgType(self, in_raw_message):
        start = in_raw_message.find(self.START_CHAR_RX)
        if(start == -1):
            start = 0
        data = self.__MSG_TYPE_DICT.get(in_raw_message[start], '')
        return data

    def getMsgPayload(self, in_raw_message):
        start = in_raw_message.find(self.START_CHAR_RX)
        if(start == -1):
            start = 1
        else:
            start = start + 1
        tmpstr = '%s'%(in_raw_message[start:-2])
        return tmpstr

    def boIsTerminatorPresent(self, in_raw_message):
        return (in_raw_message[-2:] == self.TERMINATOR_STR)

    def boIsMessageValid(self,in_raw_message):
        #in_raw_message = in_raw_message.decode("ascii")
        boVal = False
        if(self.getMsgType(in_raw_message)):
            if(self.boIsTerminatorPresent(in_raw_message)):
                crc16_func = crcmod.predefined.mkCrcFun('crc-16')
                tmpvar = self.getMsgPayload(in_raw_message)
                #if(crc16_func(bytes.fromhex(tmpvar)) == 0):
                try:
                    if(sys.version[0]=='3'):
                        hexstr = bytes.fromhex(tmpvar)
                    else:
                        hexstr = tmpvar.decode('hex')
                    boVal = (crc16_func(hexstr) == 0)
                except:
                    print('Not able to perform checksum')
                    boVal = False
            else:
                print('Terminator is not present')
        else:
            print('Message type is not correct')
        return boVal

    def getMsgCrc(self,in_str):
         bo = BitOperations()
         return bo.get_crc16(self.getMsgPayload(in_str)[:-4])

class CID_PING_SEND():

    def __init__(self, beacon_id = 'BEACON_DEFAULT', msgtype = 'MSG_REQX'):
        self.msg_id_e = CID_E('CID_PING_SEND')
        self.dest_id_e = BID_E(beacon_id)
        self.msg_type_e = AMSGTYPE_E(msgtype)
        self.csum = 0
        # Response Message parameters
        self.resp_msg_id_e = CID_E('CID_DEFAULT')
        self.resp_msg_cst_e = CST_E('CST_DEFAULT')
        self.resp_msg_bid_e = BID_E('BEACON_DEFAULT')
        self.resp_msg_csum = 0

    def encode_frame(self):
        bo = BitOperations()
        tmpstr = ''
        # Encode data into a packet ready to be transmitted
        tmpstr += self.msg_id_e.encode_data_hex()
        tmpstr += self.dest_id_e.encode_data_hex()
        tmpstr += self.msg_type_e.encode_data_hex()
        # Compute the frame checksum
        self.csum = bo.get_crc16(tmpstr)
        tmpstr += self.csum
        # Append the transmission start character
        tmpstr = MsgOperation().START_CHAR_TX + tmpstr
        return tmpstr

    def decode_response(self,rawdata):
        bo = BitOperations()
        tmpstr = ''
        # Decode the response
        self.resp_msg_id_e.decode_data(rawdata[1:3])
        self.resp_msg_cst_e.decode_data(rawdata[3:5])
        self.resp_msg_bid_e.decode_data(rawdata[5:7])
        self.resp_msg_csum = bo.hexstrtodec(rawdata[-4:])
        return [self.resp_msg_id_e.Cid, self.resp_msg_cst_e.Cst, self.resp_msg_bid_e.Bid, self.resp_msg_csum]

class CID_PING_REQ():

    def __init__(self):
        self.msg_id_e = CID_E('CID_PING_REQ')
        self.acofix_t = ACOFIX_T()
        self.csum = 0
        # Response Message parameters
        self.resp_msg_msg_id_e = CID_E('CID_DEFAULT')
        self.resp_msg_acofix_t = ACOFIX_T()
        self.resp_msg_csum = 0

    def encode_frame(self):
        tmpstr = ''
        # This is only a Rx message, so encoding method is forced to return null string
        return tmpstr

    def decode_response(self,rawdata):
        bo = BitOperations()
        tmplist = []
        # Decode the response
        st = 1; sz = 2;   self.resp_msg_msg_id_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.resp_msg_msg_id_e.Cid)
        st += sz; tmplist.append(self.resp_msg_acofix_t.decode_data(rawdata[st:-6]))
        # Compute incoming csum
        self.resp_msg_csum = bo.hexstrtodec(rawdata[-4:])

        return tmplist

class CID_PING_RESP():

    def __init__(self):
        self.msg_id_e = CID_E('CID_PING_RESP')
        self.acofix_t = ACOFIX_T()
        self.csum = 0
        # Response Message parameters
        self.resp_msg_msg_id_e = CID_E('CID_DEFAULT')
        self.resp_msg_acofix_t = ACOFIX_T()
        self.resp_msg_csum = 0

    def encode_frame(self):
        tmpstr = ''
        # This is only a Rx message, so encoding method is forced to return null string
        return tmpstr

    def decode_response(self,rawdata):
        bo = BitOperations()
        tmplist = []
        # Decode the response
        st = 1; sz = 2;   self.resp_msg_msg_id_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.resp_msg_msg_id_e.Cid)
        st += sz; tmplist.append(self.resp_msg_acofix_t.decode_data(rawdata[st:-6]))
        # Compute incoming csum
        self.resp_msg_csum = bo.hexstrtodec(rawdata[-4:])

        return tmplist

class CID_PING_ERROR():

    def __init__(self, beacon_id = 'BEACON_DEFAULT'):
        self.msg_id_e = CID_E('CID_PING_ERROR')
        self.cst_e = CST_E('CST_DEFAULT')
        self.dest_id_e = BID_E(beacon_id)
        self.csum = 0
        # Response Message parameters
        self.resp_msg_msg_id_e = CID_E('CID_PING_ERROR')
        self.resp_msg_cst_e = CST_E('CST_DEFAULT')
        self.resp_msg_dest_id_e = BID_E(beacon_id)
        self.resp_msg_csum = 0

    def encode_frame(self):
        bo = BitOperations()
        tmpstr = ''
        # Encode data into a packet ready to be transmitted
        tmpstr += self.msg_id_e.encode_data_hex()
        tmpstr += self.cst_e.encode_data_hex()
        tmpstr += self.dest_id_e.encode_data_hex()
        # Compute the frame checksum
        self.csum = bo.get_crc16(tmpstr)
        tmpstr += self.csum
        # Append the transmission start character
        tmpstr = MsgOperation().START_CHAR_TX + tmpstr
        return tmpstr

    def decode_response(self,rawdata):
        bo = BitOperations()
        tmpstr = ''
        # Decode the response
        self.resp_msg_msg_id_e.decode_data(rawdata[1:3])
        self.resp_msg_cst_e.decode_data(rawdata[3:5])
        self.resp_msg_dest_id_e.decode_data(rawdata[5:7])
        self.resp_msg_csum = bo.hexstrtodec(rawdata[-4:])
        return [self.resp_msg_msg_id_e.Cid, self.resp_msg_cst_e.Cst, self.resp_msg_dest_id_e.Bid, self.resp_msg_csum]

class CID_STATUS():

    def __init__(self):
        # Tx Variables
        self.msg_id_e = CID_E('CID_STATUS')
        self.status_output_t = STATUS_BITS_T([0, 0, 0, 1, 1, 1, 1, 1])
        # Rx variables
        self.resp_msg_id_e = CID_E('CID_STATUS')
        self.resp_msg_status_output_t = STATUS_BITS_T([0, 0, 0, 1, 1, 1, 1, 1])
        self.resp_msg_timestamp_u64 = 0
        # Environmental Fields
        self.resp_msg_env_supply_u16 = 0         # milli-volts
        self.resp_msg_env_temp_i16 = 0           # deci-celsius
        self.resp_msg_env_pressure_i32 = 0       # milli-bars
        self.resp_msg_env_depth_i32 = 0          # deci-meters
        self.resp_msg_env_vos_u16 = 0            # deci-meters/second
        # Attitude fields
        self.resp_msg_att_yaw_i16 = 0            # deci-degrees
        self.resp_msg_att_pitch_i16 = 0          # deci-degrees
        self.resp_msg_att_roll_i16 = 0           # deci-degrees
        # Magnetometer calibration and status fields
        self.resp_msg_mag_cal_buf_u8 = 0         # percentage
        self.resp_msg_mag_cal_valid_bo = False   # True: magnetic calibration has been computed
        self.resp_msg_mag_cal_age_u32 = 0        # seconds since last calibration
        self.resp_msg_mag_cal_fit_u8 = 0         # percentage, 100% perfect calibration
        # Accelerometer calibration fields
        self.resp_msg_acc_lim_min_x_i16 = 0      # Raw value that represents +/- 1G
        self.resp_msg_acc_lim_min_y_i16 = 0      # Raw value that represents +/- 1G
        self.resp_msg_acc_lim_min_z_i16 = 0      # Raw value that represents +/- 1G
        self.resp_msg_acc_lim_max_x_i16 = 0      # Raw value that represents +/- 1G
        self.resp_msg_acc_lim_max_y_i16 = 0      # Raw value that represents +/- 1G
        self.resp_msg_acc_lim_max_z_i16 = 0      # Raw value that represents +/- 1G
        # Raw AHRS sensor data fields
        self.resp_msg_ahrs_raw_acc_x_i16 = 0     # last raw sensor value measured
        self.resp_msg_ahrs_raw_acc_y_i16 = 0     # last raw sensor value measured
        self.resp_msg_ahrs_raw_acc_z_i16 = 0     # last raw sensor value measured
        self.resp_msg_ahrs_raw_mag_x_i16 = 0     # last raw sensor value measured
        self.resp_msg_ahrs_raw_mag_y_i16 = 0     # last raw sensor value measured
        self.resp_msg_ahrs_raw_mag_z_i16 = 0     # last raw sensor value measured
        self.resp_msg_ahrs_raw_gyro_x_i16 = 0    # last raw sensor value measured
        self.resp_msg_ahrs_raw_gyro_y_i16 = 0    # last raw sensor value measured
        self.resp_msg_ahrs_raw_gyro_z_i16 = 0    # last raw sensor value measured
        # Compensated AHRS sensor data fields
        self.resp_msg_ahrs_comp_acc_x_f = 0.0    # Sensor reading after the calibration is applied
        self.resp_msg_ahrs_comp_acc_y_f = 0.0    # Sensor reading after the calibration is applied
        self.resp_msg_ahrs_comp_acc_z_f = 0.0    # Sensor reading after the calibration is applied
        self.resp_msg_ahrs_comp_mag_x_f = 0.0    # Sensor reading after the calibration is applied
        self.resp_msg_ahrs_comp_mag_y_f = 0.0    # Sensor reading after the calibration is applied
        self.resp_msg_ahrs_comp_mag_z_f = 0.0    # Sensor reading after the calibration is applied
        self.resp_msg_ahrs_comp_gyro_x_f = 0.0   # Sensor reading after the calibration is applied
        self.resp_msg_ahrs_comp_gyro_y_f = 0.0   # Sensor reading after the calibration is applied
        self.resp_msg_ahrs_comp_gyro_z_f = 0.0   # Sensor reading after the calibration is applied

        self.resp_msg_csum = 0

    def encode_frame(self):
        bo = BitOperations()
        tmpstr = ''
        # Encode data into a packet ready to be transmitted
        tmpstr += self.msg_id_e.encode_data_hex()
        tmpstr += self.status_output_t.encode_data_hex()
        # Compute the frame checksum
        self.csum = bo.get_crc16(tmpstr)
        tmpstr += self.csum
        # Append the transmission start character
        tmpstr = MsgOperation().START_CHAR_TX + tmpstr
        return tmpstr

    def decode_response(self,rawdata):
        bo = BitOperations()
        tmplist = []
        # Decode the response
        st = 1; sz = 2;    self.resp_msg_id_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.resp_msg_id_e.Cid)
        st += sz; sz = 2; self.resp_msg_status_output_t.decode_data(rawdata[st:st+sz]); tmplist.append(self.resp_msg_status_output_t.byte)
        st += sz; sz = 16;  self.resp_msg_timestamp_u64 = bo.hexstrtodec(bo.dectohexstr(bo.swap_ui64(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_timestamp_u64)
        # Environmental Fields
        if self.resp_msg_status_output_t.environment == 1:
            st += sz; sz = 4;  self.resp_msg_env_supply_u16 = bo.hexstrtodec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_env_supply_u16)             # milli-volts
            st += sz; sz = 4;  self.resp_msg_env_temp_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_env_temp_i16)             # deci-celsius
            st += sz; sz = 8;  self.resp_msg_env_pressure_i32 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui32(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_env_pressure_i32)     # milli-bars
            st += sz; sz = 8;  self.resp_msg_env_depth_i32 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui32(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_env_depth_i32)           # deci-meters
            st += sz; sz = 4;  self.resp_msg_env_vos_u16 = bo.hexstrtodec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_env_vos_u16)                   # deci-meters/second
        # Attitude fields
        if self.resp_msg_status_output_t.attitude == 1:
            st += sz; sz = 4;  self.resp_msg_att_yaw_i16 =  bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_att_yaw_i16)               # deci-degrees
            st += sz; sz = 4;  self.resp_msg_att_pitch_i16 =  bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_att_pitch_i16)           # deci-degrees
            st += sz; sz = 4;  self.resp_msg_att_roll_i16 =  bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_att_roll_i16)             # deci-degrees
        # Magnetometer calibration and status fields
        if self.resp_msg_status_output_t.mag_cal == 1:
            st += sz; sz = 2;  self.resp_msg_mag_cal_buf_u8 = bo.hexstrtodec(rawdata[st:st+sz]); tmplist.append(self.resp_msg_mag_cal_buf_u8)             # percentage
            st += sz; sz = 2;  self.resp_msg_mag_cal_valid_bo = bo.hexstrtodec(rawdata[st:st+sz]); tmplist.append(self.resp_msg_mag_cal_valid_bo)         # True: magnetic calibration has been computed
            st += sz; sz = 8;  self.resp_msg_mag_cal_age_u32 = bo.hexstrtodec(bo.dectohexstr(bo.swap_ui32(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_mag_cal_age_u32)           # seconds since last calibration
            st += sz; sz = 2;  self.resp_msg_mag_cal_fit_u8 = bo.hexstrtodec(rawdata[st:st+sz]); tmplist.append(self.resp_msg_mag_cal_fit_u8)             # percentage, 100% perfect calibration
        # Accelerometer calibration fields
        if self.resp_msg_status_output_t.acc_cal == 1:
            st += sz; sz = 4;  self.resp_msg_acc_lim_min_x_i16 =  bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_acc_lim_min_x_i16)      # Raw value that represents +/- 1G
            st += sz; sz = 4;  self.resp_msg_acc_lim_min_y_i16 =  bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_acc_lim_min_y_i16)      # Raw value that represents +/- 1G
            st += sz; sz = 4;  self.resp_msg_acc_lim_min_z_i16 =  bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_acc_lim_min_z_i16)      # Raw value that represents +/- 1G
            st += sz; sz = 4;  self.resp_msg_acc_lim_max_x_i16 =  bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_acc_lim_max_x_i16)      # Raw value that represents +/- 1G
            st += sz; sz = 4;  self.resp_msg_acc_lim_max_y_i16 =  bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_acc_lim_max_y_i16)      # Raw value that represents +/- 1G
            st += sz; sz = 4;  self.resp_msg_acc_lim_max_z_i16 =  bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_acc_lim_max_z_i16)      # Raw value that represents +/- 1G
        # Raw AHRS sensor data fields
        if self.resp_msg_status_output_t.ahrs_raw_data == 1:
            st += sz; sz = 4;  self.resp_msg_ahrs_raw_acc_x_i16 =  bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_ahrs_raw_acc_x_i16)      # last raw sensor value measured
            st += sz; sz = 4;  self.resp_msg_ahrs_raw_acc_y_i16 =  bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_ahrs_raw_acc_y_i16)      # last raw sensor value measured
            st += sz; sz = 4;  self.resp_msg_ahrs_raw_acc_z_i16 =  bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_ahrs_raw_acc_z_i16)      # last raw sensor value measured
            st += sz; sz = 4;  self.resp_msg_ahrs_raw_mag_x_i16 =  bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_ahrs_raw_mag_x_i16)      # last raw sensor value measured
            st += sz; sz = 4;  self.resp_msg_ahrs_raw_mag_y_i16 =  bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_ahrs_raw_mag_y_i16)      # last raw sensor value measured
            st += sz; sz = 4;  self.resp_msg_ahrs_raw_mag_z_i16 =  bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_ahrs_raw_mag_z_i16)      # last raw sensor value measured
            st += sz; sz = 4;  self.resp_msg_ahrs_raw_gyro_x_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_ahrs_raw_gyro_x_i16)    # last raw sensor value measured
            st += sz; sz = 4;  self.resp_msg_ahrs_raw_gyro_y_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_ahrs_raw_gyro_y_i16)    # last raw sensor value measured
            st += sz; sz = 4;  self.resp_msg_ahrs_raw_gyro_z_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_ahrs_raw_gyro_z_i16)    # last raw sensor value measured
        # Compensated AHRS sensor data fields
        if self.resp_msg_status_output_t.ahrs_comp_data == 1:
            st += sz; sz = 8;  self.resp_msg_ahrs_comp_acc_x_f = bo.hexstrtofloat(rawdata[st:st+sz]); tmplist.append(self.resp_msg_ahrs_comp_acc_x_f)     # Sensor reading after the calibration is applied
            st += sz; sz = 8;  self.resp_msg_ahrs_comp_acc_y_f = bo.hexstrtofloat(rawdata[st:st+sz]); tmplist.append(self.resp_msg_ahrs_comp_acc_y_f)     # Sensor reading after the calibration is applied
            st += sz; sz = 8;  self.resp_msg_ahrs_comp_acc_z_f = bo.hexstrtofloat(rawdata[st:st+sz]); tmplist.append(self.resp_msg_ahrs_comp_acc_z_f)     # Sensor reading after the calibration is applied
            st += sz; sz = 8;  self.resp_msg_ahrs_comp_mag_x_f = bo.hexstrtofloat(rawdata[st:st+sz]); tmplist.append(self.resp_msg_ahrs_comp_mag_x_f)     # Sensor reading after the calibration is applied
            st += sz; sz = 8;  self.resp_msg_ahrs_comp_mag_y_f = bo.hexstrtofloat(rawdata[st:st+sz]); tmplist.append(self.resp_msg_ahrs_comp_mag_y_f)     # Sensor reading after the calibration is applied
            st += sz; sz = 8;  self.resp_msg_ahrs_comp_mag_z_f = bo.hexstrtofloat(rawdata[st:st+sz]); tmplist.append(self.resp_msg_ahrs_comp_mag_z_f)     # Sensor reading after the calibration is applied
            st += sz; sz = 8;  self.resp_msg_ahrs_comp_gyro_x_f = bo.hexstrtofloat(rawdata[st:st+sz]); tmplist.append(self.resp_msg_ahrs_comp_gyro_x_f)   # Sensor reading after the calibration is applied
            st += sz; sz = 8;  self.resp_msg_ahrs_comp_gyro_y_f = bo.hexstrtofloat(rawdata[st:st+sz]); tmplist.append(self.resp_msg_ahrs_comp_gyro_y_f)   # Sensor reading after the calibration is applied
            st += sz; sz = 8;  self.resp_msg_ahrs_comp_gyro_z_f = bo.hexstrtofloat(rawdata[st:st+sz]); tmplist.append(self.resp_msg_ahrs_comp_gyro_z_f)   # Sensor reading after the calibration is applied

        self.resp_msg_csum = bo.hexstrtodec(rawdata[-4:])

        return tmplist

class CID_XCVR_FIX:
    def __init__(self):
        self.msg_id_e = CID_E('CID_XCVR_FIX')
        self.acofix_t = ACOFIX_T()
        self.csum = 0
        # Response Message parameters
        self.resp_msg_msg_id_e = CID_E('CID_XCVR_FIX')
        self.resp_msg_acofix_t = ACOFIX_T()
        self.resp_msg_csum = 0

    def encode_frame(self):
        bo = BitOperations()
        tmpstr = ''
        # Encode data into a packet ready to be transmitted
        tmpstr += self.msg_id_e.encode_data_hex()
        tmpstr += self.acofix_t.encode_data_hex()
        # Compute the frame checksum
        self.csum = bo.get_crc16(tmpstr)
        tmpstr += self.csum
        # Append the transmission start character
        tmpstr = MsgOperation().START_CHAR_TX + tmpstr
        return tmpstr

    def decode_response(self,rawdata):
        bo = BitOperations()
        tmplist = []
        # Decode the response
        st = 1; sz = 2;   self.resp_msg_msg_id_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.resp_msg_msg_id_e.Cid)
        st += sz; tmplist.append(self.resp_msg_acofix_t.decode_data(rawdata[st:-6]))

        self.resp_msg_csum = bo.hexstrtodec(rawdata[-4:])

        return tmplist

class CID_XCVR_USBL:
    def __init__(self):
        self.msg_id_e = CID_E('CID_XCVR_USBL')
        self.xcor_sig_peak_f = 0.0
        self.xcor_threshold_f = 0.0
        self.xcor_cross_point_u16 = 0
        self.xcor_cross_mag_f = 0.0
        self.xcor_detect_u16 = 0
        self.xcor_length_u16 = 140
        self.xcor_data_af = [0.0]*self.xcor_length_u16
        self.channels_u8 = 4
        self.channel_rssi_au16 = [0]*self.channels_u8
        self.baselines_u8 = 6
        self.phase_angle_af = [0.0]*self.baselines_u8
        self.signal_azimuth_i16 = 0
        self.signal_elevation_i16 = 0
        self.signal_fit_error_f = 0.0
        self.csum = 0
        # Response Message parameters
        self.resp_msg_msg_id_e = CID_E('CID_XCVR_USBL')
        self.resp_msg_xcor_sig_peak_f = 0.0
        self.resp_msg_xcor_threshold_f = 0.0
        self.resp_msg_xcor_cross_point_u16 = 0
        self.resp_msg_xcor_cross_mag_f = 0.0
        self.resp_msg_xcor_detect_u16 = 0
        self.resp_msg_xcor_length_u16 = 140
        self.resp_msg_xcor_data_af = [0.0]*self.resp_msg_xcor_length_u16
        self.resp_msg_channels_u8 = 4
        self.resp_msg_channel_rssi_au16 = [0]*self.resp_msg_channels_u8
        self.resp_msg_baselines_u8 = 6
        self.resp_msg_phase_angle_af = [0.0]*self.baselines_u8
        self.resp_msg_signal_azimuth_i16 = 0
        self.resp_msg_signal_elevation_i16 = 0
        self.resp_msg_signal_fit_error_f = 0.0
        self.resp_msg_csum = 0

    def encode_frame(self):
        bo = BitOperations()
        tmpstr = ''
        # Encode data into a packet ready to be transmitted
        tmpstr += self.msg_id_e.encode_data_hex()
        tmpstr += bo.floattohexstr(self.xcor_sig_peak_f)
        tmpstr += bo.floattohexstr(self.xcor_threshold_f)
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.xcor_cross_point_u16),4)
        tmpstr += bo.floattohexstr(self.xcor_cross_mag_f)
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.xcor_detect_u16),4)
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.xcor_length_u16),4)
        tmpstr += bo.floatlisttohexstr(self.xcor_data_af)
        tmpstr += bo.dectohexstr(self.channels_u8,2)
        tmpstr += bo.declisttohexstr(self.channel_rssi_au16,4)
        tmpstr += bo.dectohexstr(self.baselines_u8,2)
        tmpstr += bo.floatlisttohexstr(self.phase_angle_af)
        tmpstr += bo.dectohexstr(bo.swap_i16(self.signal_azimuth_i16),4)
        tmpstr += bo.dectohexstr(bo.swap_i16(self.signal_elevation_i16),4)
        tmpstr += bo.floattohexstr(self.signal_fit_error_f)
        # Compute the frame checksum
        self.csum = bo.get_crc16(tmpstr)
        tmpstr += self.csum
        # Append the transmission start character
        tmpstr = MsgOperation().START_CHAR_TX + tmpstr
        return tmpstr

    def decode_response(self,rawdata):
        bo = BitOperations()
        tmplist = []
        # Decode the response
        st = 1; sz = 2;    self.resp_msg_msg_id_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.resp_msg_msg_id_e.Cid)
        st += sz; sz = 8;  self.xcor_sig_peak_f = bo.hexstrtofloat(rawdata[st:st+sz]); tmplist.append(self.xcor_sig_peak_f)
        st += sz; sz = 8;  self.resp_msg_xcor_threshold_f = bo.hexstrtofloat(rawdata[st:st+sz]); tmplist.append(self.resp_msg_xcor_threshold_f)
        st += sz; sz = 4;  self.resp_msg_xcor_cross_point_u16 = bo.hexstrtodec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_xcor_cross_point_u16)
        st += sz; sz = 8;  self.resp_msg_xcor_cross_mag_f = bo.hexstrtofloat(rawdata[st:st+sz]); tmplist.append(self.resp_msg_xcor_cross_mag_f)
        st += sz; sz = 4;  self.resp_msg_xcor_detect_u16 = bo.hexstrtodec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_xcor_detect_u16)
        st += sz; sz = 4;  self.resp_msg_xcor_length_u16 = bo.hexstrtodec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_xcor_length_u16)
        st += sz; sz = 8*self.resp_msg_xcor_length_u16;  self.resp_msg_xcor_data_af = bo.hexstrtofloatlist(rawdata[st:st+sz]); tmplist.append(self.resp_msg_xcor_data_af)
        st += sz; sz = 2;  self.resp_msg_channels_u8 = bo.hexstrtodec(rawdata[st:st+sz]); tmplist.append(self.resp_msg_channels_u8)
        st += sz; sz = 4*self.resp_msg_channels_u8;  self.resp_msg_channel_rssi_au16 = bo.hexstrtodeclist(rawdata[st:st+sz],4); tmplist.append(self.resp_msg_channel_rssi_au16)
        st += sz; sz = 2;  self.resp_msg_baselines_u8 = bo.hexstrtodec(rawdata[st:st+sz]); tmplist.append(self.resp_msg_baselines_u8)
        st += sz; sz = 8*self.resp_msg_baselines_u8;  self.resp_msg_phase_angle_af = bo.hexstrtofloatlist(rawdata[st:st+sz]); tmplist.append(self.resp_msg_phase_angle_af)
        st += sz; sz = 4;  self.resp_msg_signal_azimuth_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_signal_azimuth_i16)
        st += sz; sz = 4;  self.resp_msg_signal_elevation_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_signal_elevation_i16)
        st += sz; sz = 8;  self.resp_msg_signal_fit_error_f = bo.hexstrtofloat(rawdata[st:st+sz]); tmplist.append(self.resp_msg_signal_fit_error_f)
        self.resp_msg_csum = bo.hexstrtodec(rawdata[-4:])
        return tmplist

class CID_SETTINGS_GET:
    def __init__(self):
        self.msg_id_e = CID_E('CID_SETTINGS_GET')
        self.csum = 0
        # Response Message parameters
        self.resp_msg_msg_id_e = CID_E('CID_SETTINGS_GET')
        self.resp_msg_settings_t = SETTINGS_T()
        self.resp_msg_csum = 0

    def encode_frame(self):
        bo = BitOperations()
        tmpstr = ''
        # Encode data into a packet ready to be transmitted
        tmpstr += self.msg_id_e.encode_data_hex()
        # Compute the frame checksum
        self.csum = bo.get_crc16(tmpstr)
        tmpstr += self.csum
        # Append the transmission start character
        tmpstr = MsgOperation().START_CHAR_TX + tmpstr
        return tmpstr

    def decode_response(self,rawdata):
        bo = BitOperations()
        tmplist = []
        # Decode the response
        st = 1; sz = 2;    self.resp_msg_msg_id_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.resp_msg_msg_id_e.Cid)
        st += sz; sz = -4;  self.resp_msg_settings_t.decode_data(rawdata[st:sz]);  tmplist.append(self.resp_msg_settings_t)
        self.resp_msg_csum = bo.hexstrtodec(rawdata[-4:])
        return tmplist

class CID_DAT_SEND():

    def __init__(self, beacon_id = 'BEACON_DEFAULT', msgtype = 'MSG_OWAY', packet_data = '170.5'):
        self.msg_id_e = CID_E('CID_DAT_SEND')
        self.dest_id_e = BID_E(beacon_id)
        self.msg_type_e = AMSGTYPE_E(msgtype)
        self.packet_len = len(packet_data)
        self.packet_data = packet_data
        self.csum = 0
        # Response Message parameters
        self.resp_msg_id_e = CID_E('CID_DEFAULT')
        self.resp_msg_cst_e = CST_E('CST_DEFAULT')
        self.resp_msg_bid_e = BID_E('BEACON_DEFAULT')
        self.resp_msg_csum = 0

    def encode_frame(self):
        bo = BitOperations()
        tmpstr = ''
        # Encode data into a packet ready to be transmitted
        tmpstr += self.msg_id_e.encode_data_hex()
        tmpstr += self.dest_id_e.encode_data_hex()
        tmpstr += self.msg_type_e.encode_data_hex()
        tmpstr += bo.dectohexstr(self.packet_len,2)
        tmpstr += bo.strtohexascii(self.packet_data)
        # Compute the frame checksum
        self.csum = bo.get_crc16(tmpstr)
        tmpstr += self.csum
        # Append the transmission start character
        tmpstr = MsgOperation().START_CHAR_TX + tmpstr
        return tmpstr

    def decode_response(self,rawdata):
        bo = BitOperations()
        tmpstr = ''
        # Decode the response
        self.resp_msg_id_e.decode_data(rawdata[1:3])
        self.resp_msg_cst_e.decode_data(rawdata[3:5])
        self.resp_msg_bid_e.decode_data(rawdata[5:7])
        self.resp_msg_csum = bo.hexstrtodec(rawdata[-4:])
        return [self.resp_msg_id_e.Cid, self.resp_msg_cst_e.Cst, self.resp_msg_bid_e.Bid, self.resp_msg_csum]

class CID_DAT_RECEIVE:
    def __init__(self, beacon_id = 'BEACON_DEFAULT', packet_data = 'NONE'):
        self.msg_id_e = CID_E('CID_DAT_RECEIVE')
        self.acofix_t = ACOFIX_T()
        self.ack_flag = 0
        self.packet_len = 0
        self.packet_data = packet_data
        self.csum = 0
        # Response Message parameters
        self.resp_msg_msg_id_e = CID_E('CID_DAT_RECEIVE')
        self.resp_msg_acofix_t = ACOFIX_T()
        self.resp_msg_ack_flag = 0
        self.resp_msg_packet_len = 0
        self.resp_msg_packet_data = 'NONE'
        self.resp_msg_csum = 0

    def encode_frame(self):
        bo = BitOperations()
        tmpstr = ''
        # Encode data into a packet ready to be transmitted
        return tmpstr

    def decode_response(self,rawdata):
        bo = BitOperations()
        tmplist = []
        # Decode the response
        st = 1; sz = 2;   self.resp_msg_msg_id_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.resp_msg_msg_id_e.Cid)
        st += sz; tmplist.append(self.resp_msg_acofix_t.decode_data(rawdata[st:]))
        st += self.resp_msg_acofix_t.rcvd_length; sz = 2;  self.resp_msg_ack_flag = bo.hexstrtodec(rawdata[st:st+sz]); tmplist.append(self.resp_msg_ack_flag)
        st += sz; sz = 2;  self.resp_msg_packet_len = bo.hexstrtodec(rawdata[st:st+sz]); tmplist.append(self.resp_msg_packet_len)
        st += sz; sz = 2*self.resp_msg_packet_len;  self.resp_msg_packet_data = bo.hexstrtoasciistring(rawdata[st:st+sz]); tmplist.append(self.resp_msg_packet_data)
        self.resp_msg_csum = bo.hexstrtodec(rawdata[-4:])
        return tmplist

class CID_DAT_QUEUE_SET:
     def __init__(self, beacon_id = 'BEACON_DEFAULT', packet_data = 'NONE'):
         self.msg_id_e = CID_E('CID_DAT_QUEUE_SET')
         self.dest_bid_e = BID_E(beacon_id)
         self.packet_len = len(packet_data)
         self.packet_data = packet_data
         self.csum = 0
         # Response Message parameters
         self.resp_msg_msg_id_e = CID_E('CID_DAT_QUEUE_SET')
         self.resp_msg_cst_e = CST_E('CST_DEFAULT')
         self.resp_msg_dest_bid_e = BID_E('BEACON_DEFAULT')
         self.resp_msg_packet_len = 0
         self.resp_msg_csum = 0

     def encode_frame(self):
         bo = BitOperations()
         tmpstr = ''
         # Encode data into a packet ready to be transmitted
         tmpstr += self.msg_id_e.encode_data_hex()
         tmpstr += self.dest_bid_e.encode_data_hex()
         tmpstr += bo.dectohexstr(self.packet_len,2)
         tmpstr += bo.strtohexascii(self.packet_data)
         # Compute the frame checksum
         self.csum = bo.get_crc16(tmpstr)
         tmpstr += self.csum
         # Append the transmission start character
         tmpstr = MsgOperation().START_CHAR_TX + tmpstr
         return tmpstr

     def decode_response(self,rawdata):
         bo = BitOperations()
         tmplist = []
         # Decode the response
         st = 1; sz = 2;   self.resp_msg_msg_id_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.resp_msg_msg_id_e.Cid)
         st += sz; sz = 2; self.resp_msg_cst_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.resp_msg_cst_e.Cst)
         st += sz; sz = 2;   self.resp_msg_dest_bid_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.resp_msg_dest_bid_e.Bid)
         st += sz; sz = 2;  self.resp_msg_packet_len = bo.hexstrtodec(rawdata[st:st+sz]); tmplist.append(self.resp_msg_packet_len)
         self.resp_msg_csum = bo.hexstrtodec(rawdata[-4:])
         return tmplist

class CID_XCVR_RX_RESP:
    def __init__(self):
        self.msg_id_e = CID_E('CID_XCVR_RX_RESP')
        self.acofix_t = ACOFIX_T()
        self.aco_msg_t = ACOMSG_T()
        self.csum = 0
        # Response Message parameters
        self.resp_msg_msg_id_e = CID_E('CID_XCVR_RX_RESP')
        self.resp_msg_acofix_t = ACOFIX_T()
        self.resp_msg_aco_msg_t = ACOMSG_T()
        self.resp_msg_csum = 0

    def encode_frame(self):
        bo = BitOperations()
        tmpstr = ''
        # Encode data into a packet ready to be transmitted
        return tmpstr

    def decode_response(self,rawdata):
        bo = BitOperations()
        tmplist = []
        # Decode the response
        st = 1; sz = 2;   self.resp_msg_msg_id_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.resp_msg_msg_id_e.Cid)
        st += sz;                                 tmplist.append(self.resp_msg_acofix_t.decode_data(rawdata[st:]))
        st += self.resp_msg_acofix_t.rcvd_length; tmplist.append(self.resp_msg_aco_msg_t.decode_data(rawdata[st:]))
        self.resp_msg_csum = bo.hexstrtodec(rawdata[-4:])
        return tmplist

class CID_XCVR_RX_REQ:
    def __init__(self):
        self.msg_id_e = CID_E('CID_XCVR_RX_REQ')
        self.acofix_t = ACOFIX_T()
        self.aco_msg_t = ACOMSG_T()
        self.csum = 0
        # Response Message parameters
        self.resp_msg_msg_id_e = CID_E('CID_XCVR_RX_REQ')
        self.resp_msg_acofix_t = ACOFIX_T()
        self.resp_msg_aco_msg_t = ACOMSG_T()
        self.resp_msg_csum = 0

    def encode_frame(self):
        bo = BitOperations()
        tmpstr = ''
        # Encode data into a packet ready to be transmitted
        return tmpstr

    def decode_response(self,rawdata):
        bo = BitOperations()
        tmplist = []
        # Decode the response
        st = 1; sz = 2;   self.resp_msg_msg_id_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.resp_msg_msg_id_e.Cid)
        st += sz;                                 tmplist.append(self.resp_msg_acofix_t.decode_data(rawdata[st:]))
        st += self.resp_msg_acofix_t.rcvd_length; tmplist.append(self.resp_msg_aco_msg_t.decode_data(rawdata[st:]))
        self.resp_msg_csum = bo.hexstrtodec(rawdata[-4:])
        return tmplist

class CID_XCVR_ANALYSE:
    def __init__(self):
        self.msg_id_e = CID_E('CID_XCVR_ANALYSE')
        self.csum = 0
        # Response Message parameters
        self.resp_msg_msg_id_e = CID_E('CID_XCVR_ANALYSE')
        self.resp_msg_cst_e = CST_E('CST_DEFAULT')
        self.resp_msg_adc_mean_i16 = 0
        self.resp_msg_adc_pkpk_i16 = 0
        self.resp_msg_adc_rms_u32 = 0
        self.resp_msg_rx_lvl_pkpk_i16 = 0
        self.resp_msg_rx_lvl_rms_i16 = 0
        self.resp_msg_csum = 0

    def encode_frame(self):
        bo = BitOperations()
        tmpstr = ''
        # Encode data into a packet ready to be transmitted
        tmpstr += self.msg_id_e.encode_data_hex()
        # Compute the frame checksum
        self.csum = bo.get_crc16(tmpstr)
        tmpstr += self.csum
        # Append the transmission start character
        tmpstr = MsgOperation().START_CHAR_TX + tmpstr
        return tmpstr

    def decode_response(self,rawdata):
        bo = BitOperations()
        tmplist = []
        # Decode the response
        st = 1; sz = 2;   self.resp_msg_msg_id_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.resp_msg_msg_id_e.Cid)
        st += sz; sz = 2;   self.resp_msg_cst_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.resp_msg_cst_e.Cst)
        st += sz; sz = 4;  self.resp_msg_adc_mean_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_adc_mean_i16)
        st += sz; sz = 4;  self.resp_msg_adc_pkpk_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_adc_pkpk_i16)
        st += sz; sz = 8;  self.resp_msg_adc_rms_u32 = bo.hexstrtodec(bo.dectohexstr(bo.swap_ui32(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_adc_rms_u32)
        st += sz; sz = 4;  self.resp_msg_rx_lvl_pkpk_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_rx_lvl_pkpk_i16)
        st += sz; sz = 4;  self.resp_msg_rx_lvl_rms_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_rx_lvl_rms_i16)
        self.resp_msg_csum = bo.hexstrtodec(rawdata[-4:])
        return tmplist


class CID_NAV_QUERY_SEND:
    def __init__(self, beacon_id = 'BEACON_DEFAULT', packet_data = 'HelloWorld'):
        self.msg_id_e = CID_E('CID_NAV_QUERY_SEND')
        self.dest_id_e = BID_E(beacon_id)
        self.query_flags_t = NAV_QUERY_T(bitfield = [1, 1, 1, 1, 1, 1, 1, 1])
        self.packet_len_u8 = len(packet_data)
        self.packet_data_au8 = packet_data
        self.csum = 0
        # Response Message parameters
        self.resp_msg_msg_id_e = CID_E('CID_NAV_QUERY_SEND')
        self.resp_msg_cst_e = CST_E('CST_DEFAULT')
        self.resp_msg_dest_bid_e = BID_E(beacon_id)
        self.resp_msg_csum = 0

    def encode_frame(self):
        bo = BitOperations()
        tmpstr = ''
        # Encode data into a packet ready to be transmitted
        tmpstr += self.msg_id_e.encode_data_hex()
        tmpstr += self.dest_id_e.encode_data_hex()
        tmpstr += self.query_flags_t.encode_data_hex()
        pk_sz = bo.dectohexstr(self.packet_len_u8,2)
        tmpstr += pk_sz
        if(pk_sz > 0):
            tmpstr += bo.strtohexascii(self.packet_data_au8)
        # Compute the frame checksum
        self.csum = bo.get_crc16(tmpstr)
        tmpstr += self.csum
        # Append the transmission start character
        tmpstr = MsgOperation().START_CHAR_TX + tmpstr
        return tmpstr

    def decode_response(self,rawdata):
        bo = BitOperations()
        tmplist = []
        # Decode the response
        st = 1; sz = 2;   self.resp_msg_msg_id_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.resp_msg_msg_id_e.Cid)
        st += sz; sz = 2;   self.resp_msg_cst_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.resp_msg_cst_e.Cst)
        st += sz; sz = 2;   self.resp_msg_bid_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.resp_msg_bid_e.Bid)
        self.resp_msg_csum = bo.hexstrtodec(rawdata[-4:])
        return tmplist

class CID_NAV_QUERY_REQ:
    def __init__(self, beacon_id = 'BEACON_DEFAULT'):
        self.resp_msg_msg_id_e = CID_E('CID_NAV_QUERY_REQ')
        self.resp_msg_acofix_t = ACOFIX_T()
        self.resp_msg_query_flags_t = NAV_QUERY_T()
        self.resp_msg_packet_len_u8 = 0
        self.resp_msg_packet_data_au8 = []
        self.resp_msg_local_flag_bo = False
        self.resp_msg_csum = 0

    def encode_frame(self):
        bo = BitOperations()
        tmpstr = ''
        return tmpstr

    def decode_response(self,rawdata):
        bo = BitOperations()
        tmplist = []
        # Decode the response
        st = 1; sz = 2;   self.resp_msg_msg_id_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.resp_msg_msg_id_e.Cid)
        st += sz;         tmplist.append(self.resp_msg_acofix_t.decode_data(rawdata[st:]))
        st += self.resp_msg_acofix_t.rcvd_length; sz = 2; tmplist.append(self.resp_msg_query_flags_t.decode_data(rawdata[st:st+sz]))
        st += sz; sz = 2; self.resp_msg_packet_len_u8 = bo.hexstrtodec(rawdata[st:st+sz]); tmplist.append(self.resp_msg_packet_len_u8)
        st += sz; sz = 2*self.resp_msg_packet_len_u8;  self.resp_msg_packet_data_au8 = bo.hexstrtoasciistring(rawdata[st:st+sz]); tmplist.append(self.resp_msg_packet_data_au8)
        st += sz; sz = 2; self.resp_msg_local_flag_bo = (bo.hexstrtodec(rawdata[st:st+sz]) != 0)
        self.resp_msg_csum = bo.hexstrtodec(rawdata[-4:])
        return tmplist

class CID_NAV_QUERY_RESP:
    def __init__(self, beacon_id = 'BEACON_DEFAULT'):
        self.msg_id_e = CID_E('CID_NAV_QUERY_RESP')
        self.acofix_t = ACOFIX_T()
        self.query_flags_t = NAV_QUERY_T()
        # depth field
        self.remote_depth_i32 = 0
        # power supply voltage field
        self.remote_supply_u16 = 0
        # temperature field
        self.remote_temp_i16 = 0
        # attitude fields
        self.remote_yaw_i16 = 0
        self.remote_pitch_i16 = 0
        self.remote_roll_i16 = 0
        self.csum = 0
        # Response Message parameters
        self.resp_msg_msg_id_e = CID_E('CID_NAV_QUERY_RESP')
        self.resp_msg_acofix_t = ACOFIX_T()
        self.resp_msg_query_flags_t = NAV_QUERY_T()
        # depth field
        self.resp_msg_remote_depth_i32 = 0
        # power supply voltage field
        self.resp_msg_remote_supply_u16 = 0
        # temperature field
        self.resp_msg_remote_temp_i16 = 0
        # attitude fields
        self.resp_msg_remote_yaw_i16 = 0
        self.resp_msg_remote_pitch_i16 = 0
        self.resp_msg_remote_roll_i16 = 0
        # Data fields
        self.resp_msg_packet_len_u8 = 0
        self.resp_msg_packet_data_au8 = []
        self.resp_msg_local_flag_bo = False
        self.resp_msg_csum = 0

    def encode_frame(self):
        bo = BitOperations()
        tmpstr = ''
        # Encode data into a packet ready to be transmitted
        return tmpstr
        self.resp_msg_query_flags_t.qry_data
        self.resp_msg_query_flags_t.qry_attitude
        self.resp_msg_query_flags_t.qry_temp
        self.resp_msg_query_flags_t.qry_supply
        self.resp_msg_query_flags_t.qry_depth

    def decode_response(self,rawdata):
        bo = BitOperations()
        tmplist = []
        # Decode the response
        st = 1; sz = 2;   self.resp_msg_msg_id_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.resp_msg_msg_id_e.Cid)
        st += sz;         tmplist.append(self.resp_msg_acofix_t.decode_data(rawdata[st:]))
        st += self.resp_msg_acofix_t.rcvd_length; sz = 2; tmplist.append(self.resp_msg_query_flags_t.decode_data(rawdata[st:st+sz]));
        if(self.resp_msg_query_flags_t.qry_depth == 1):
            st += sz; sz = 8;  self.resp_msg_remote_depth_i32 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui32(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_remote_depth_i32)
        if(self.resp_msg_query_flags_t.qry_supply == 1):
            st += sz; sz = 4;  self.resp_msg_remote_supply_u16 = bo.hexstrtodec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_remote_supply_u16)
        if(self.resp_msg_query_flags_t.qry_temp == 1):
            st += sz; sz = 4;  self.resp_msg_remote_temp_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_remote_temp_i16)
        if(self.resp_msg_query_flags_t.qry_attitude == 1):
            st += sz; sz = 4;  self.resp_msg_remote_yaw_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_remote_yaw_i16)
            st += sz; sz = 4;  self.resp_msg_remote_pitch_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_remote_pitch_i16)
            st += sz; sz = 4;  self.resp_msg_remote_roll_i16 = bo.hexstrtosigndec(bo.dectohexstr(bo.swap_ui16(bo.hexstrtodec(rawdata[st:st+sz])),sz)); tmplist.append(self.resp_msg_remote_roll_i16)
        if(self.resp_msg_query_flags_t.qry_data == 1):
            st += sz; sz = 2;  self.resp_msg_packet_len_u8 = bo.hexstrtodec(rawdata[st:st+sz]); tmplist.append(self.resp_msg_packet_len_u8)
            st += sz; sz = 2*self.resp_msg_packet_len_u8;  self.resp_msg_packet_data_au8 = bo.hexstrtoasciistring(rawdata[st:st+sz]); tmplist.append(self.resp_msg_packet_data_au8)
        self.resp_msg_local_flag_bo = bo.hexstrtodec(rawdata[-6:-4]); tmplist.append((self.resp_msg_local_flag_bo != 0))
        self.resp_msg_csum = bo.hexstrtodec(rawdata[-4:])
        return tmplist

class CID_NAV_QUEUE_SET:
    def __init__(self, beacon_id = 'BEACON_DEFAULT', packet_data = 'HelloWorld'):
        self.msg_id_e = CID_E('CID_NAV_QUEUE_SET')
        self.dest_id_e = BID_E(beacon_id)
        self.packet_len_u8 = len(packet_data)
        self.packet_data_au8 = packet_data
        self.csum = 0
        # Response Message parameters
        self.resp_msg_msg_id_e = CID_E('CID_NAV_QUEUE_SET')
        self.resp_msg_msg_cst_e = CST_E('CST_DEFAULT')
        self.resp_msg_dest_id_e = BID_E('BEACON_DEFAULT')
        self.resp_msg_packet_len_u8 = 0
        self.resp_msg_csum = 0

    def encode_frame(self):
        bo = BitOperations()
        tmpstr = ''
        # Encode data into a packet ready to be transmitted
        tmpstr += self.msg_id_e.encode_data_hex()
        tmpstr += self.dest_id_e.encode_data_hex()
        tmpstr += bo.dectohexstr(self.packet_len_u8,2)
        tmpstr += bo.strtohexascii(self.packet_data_au8)
        # Compute the frame checksum
        self.csum = bo.get_crc16(tmpstr)
        tmpstr += self.csum
        # Append the transmission start character
        tmpstr = MsgOperation().START_CHAR_TX + tmpstr
        return tmpstr

    def decode_response(self,rawdata):
        bo = BitOperations()
        tmplist = []
        # Decode the response
        return tmplist

class CID_XCVR_TX_MSG:
    def __init__(self, beacon_id = 'BEACON_DEFAULT'):
        self.resp_msg_msg_id_e = CID_E('CID_XCVR_TX_MSG')
        self.resp_msg_aco_msg_t = ACOMSG_T()
        self.resp_msg_csum = 0

    def encode_frame(self):
        bo = BitOperations()
        tmpstr = ''
        return tmpstr

    def decode_response(self,rawdata):
        bo = BitOperations()
        tmplist = []
        # Decode the response
        st = 1; sz = 2;   self.resp_msg_msg_id_e.decode_data(rawdata[st:st+sz]);     tmplist.append(self.resp_msg_msg_id_e.Cid)
        st += sz;         tmplist.append(self.resp_msg_aco_msg_t.decode_data(rawdata[st:]))
        self.resp_msg_csum = bo.hexstrtodec(rawdata[-4:])

class CID_NAV_ERROR:
    def __init__(self):
        self.msg_id_e = CID_E('CID_NAV_ERROR')
        self.msg_cst_e = CST_E('CST_DEFAULT')
        self.dest_id_e = BID_E('BEACON_DEFAULT')
        self.csum = 0
        # Response Message parameters
    def encode_frame(self):
        bo = BitOperations()
        tmpstr = ''
        # Encode data into a packet ready to be transmitted
        return tmpstr

    def decode_response(self,rawdata):
        bo = BitOperations()
        tmplist = []
        # Decode the response
        st = 1; sz = 2;   self.msg_id_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.msg_id_e.Cid)
        st += sz; sz = 2; self.msg_cst_e.decode_data(rawdata[st:st+sz]); tmplist.append(self.msg_cst_e.Cst)
        st += sz; sz = 2; self.dest_id_e.decode_data(rawdata[st:st+sz]); tmplist.append(self.dest_id_e.Bid)
        self.resp_msg_csum = bo.hexstrtodec(rawdata[-4:])
        return tmplist

class  CID_SYS_REBOOT:
    def __init__(self):
        self.msg_id_e = CID_E('CID_SYS_REBOOT')
        self.check_u16 = int('0x6A95', 16)
        self.csum = 0
        # Response Message parameters
        self.resp_msg_msg_id_e = CID_E('CID_DEFAULT')
        self.resp_msg_msg_cst_e = CST_E('CST_DEFAULT')
        self.resp_msg_csum = 0

    def encode_frame(self):
        bo = BitOperations()
        tmpstr = ''
        # Encode data into a packet ready to be transmitted
        tmpstr += self.msg_id_e.encode_data_hex()
        tmpstr += bo.dectohexstr(bo.swap_ui16(self.check_u16),4)
        # Compute the frame checksum
        self.csum = bo.get_crc16(tmpstr)
        tmpstr += self.csum
        # Append the transmission start character
        tmpstr = MsgOperation().START_CHAR_TX + tmpstr
        return tmpstr

    def decode_response(self,rawdata):
        bo = BitOperations()
        tmplist = []
        # Decode the response
        st = 1; sz = 2;   self.msg_id_e.decode_data(rawdata[st:st+sz]);  tmplist.append(self.msg_id_e.Cid)
        st += sz; sz = 2; self.msg_cst_e.decode_data(rawdata[st:st+sz]); tmplist.append(self.msg_cst_e.Cst)
        self.resp_msg_csum = bo.hexstrtodec(rawdata[-4:])
        return tmplist


if __name__ == '__main__':
    mo = MsgOperation()

    cidDataSend = CID_DAT_SEND(beacon_id = 'BEACON_ID15', msgtype = 'MSG_OWAY', packet_data = '170.5')

    rx_frame = '$60000F401A\r\n'
    tx_frame = cidDataSend.encode_frame() + '\r\n'

    print('Tx frame: %s'%(tx_frame[:-2]))
    print('Tx Valid: %d'%(mo.boIsMessageValid(tx_frame)))
    print('Rx frame: %s'%(rx_frame[:-2]))
    print('Rx Valid: %d'%(mo.boIsMessageValid(rx_frame)))
    print('Dec fram: %s'%(str(cidDataSend.decode_response(rx_frame))))
    print('MsgType : %s'%(mo.getMsgType(rx_frame)))
    print('Cid     : %s'%(cidDataSend.resp_msg_id_e.Cid))
    print('Cst     : %s'%(cidDataSend.resp_msg_cst_e.Cst))
    print('Bid     : %s'%(cidDataSend.resp_msg_bid_e.Bid))
    print('Csum    : %d'%(cidDataSend.resp_msg_csum))

    cidStatus = CID_STATUS()

    rx_frame = '$101F7947000000000000C426B8000600000000000000423B7906ABFEDAFB0B001200000000F7FED1FF3B00F3FF5F000F0101FFD3FF3F00F1FE78FF2100E2FF0D00FEFF3AA5\r\n'
    tx_frame = cidStatus.encode_frame() + '\r\n'

    print('Tx frame  : %s'%(tx_frame[:-2]))
    print('Tx Valid  : %d'%(mo.boIsMessageValid(tx_frame)))
    print('Rx frame  : %s'%(rx_frame[:-2]))
    print('Rx Valid  : %d'%(mo.boIsMessageValid(rx_frame)))
    print('Dec fram  : %s'%(str(cidStatus.decode_response(rx_frame))))
    print('MsgType   : %s'%(mo.getMsgType(rx_frame)))
    print('Cid       : %s'%(cidStatus.resp_msg_id_e.Cid))
    print('Timestamp : %d'%(cidStatus.resp_msg_timestamp_u64))
    print('Acc X     : %d'%(cidStatus.resp_msg_ahrs_raw_acc_x_i16))
    print('Acc Y     : %d'%(cidStatus.resp_msg_ahrs_raw_acc_y_i16))
    print('Acc Z     : %d'%(cidStatus.resp_msg_ahrs_raw_acc_z_i16))
    print('Mag X     : %d'%(cidStatus.resp_msg_ahrs_raw_mag_x_i16))
    print('Mag Y     : %d'%(cidStatus.resp_msg_ahrs_raw_mag_y_i16))
    print('Mag Z     : %d'%(cidStatus.resp_msg_ahrs_raw_mag_z_i16))
    print('Gyro X    : %d'%(cidStatus.resp_msg_ahrs_raw_gyro_x_i16))
    print('Gyro Y    : %d'%(cidStatus.resp_msg_ahrs_raw_gyro_y_i16))
    print('Gyro Z    : %d'%(cidStatus.resp_msg_ahrs_raw_gyro_z_i16))
    print('Csum      : %d'%(cidStatus.resp_msg_csum))

    cidXcvrFix = CID_XCVR_FIX()

    rx_frame = '$39010F87058E06A9FC810000005C3B4005CC0300004B1D0000050004A006A406B806B3069004A801A800FDFFFDFFFFFF76B3\r\n'
    tx_frame = cidXcvrFix.encode_frame() + '\r\n'

    print('Tx frame  : %s'%(tx_frame[:-2]))
    print('Tx Valid  : %d'%(mo.boIsMessageValid(tx_frame)))
    print('Rx frame  : %s'%(rx_frame[:-2]))
    print('Rx Valid  : %d'%(mo.boIsMessageValid(rx_frame)))
    print('Dec frame : %s'%(str(cidXcvrFix.decode_response(rx_frame))))
    print('FixUsblAzimuth       : %f'%(cidXcvrFix.resp_msg_acofix_t.usbl_azimuth_i16/10))
    print('FixUsblElevation     : %f'%(cidXcvrFix.resp_msg_acofix_t.usbl_elevation_i16/10))
    print('FixPositionEasting   : %f'%(cidXcvrFix.resp_msg_acofix_t.position_easting_i16/10))
    print('FixPositionNorthing  : %f'%(cidXcvrFix.resp_msg_acofix_t.position_northing_i16/10))
    print('FixPositionDepth     : %f'%(cidXcvrFix.resp_msg_acofix_t.position_depth_i16/10))

    cidXcvrUsbl = CID_XCVR_USBL()

    rx_frame = '$3817499A4FC9B6CD4E4B0017499A4F4C008C00799C5F4C25CDC44B31F2564C3696394CF27F904B3632594CC4C7FB4B5837414B69402A4C77F0064C99D5374C84F1CC4B8F1B384C17C6324C3E48894B15D32B4CACB5EF4B32C8034C9E45994C8231064C8FC2EC4B5242164CBC2B934CA949934C0457224CB9872C4C7A317F4C834BEC4BB2FA5C4C015AD94C17C6014D2AD55F4C27A4B64B0CD4964CEC43014D9DA94E4D5782894DE049674D6863734CAB0D234D59A5654D90FB0F4D654A224D7B295B4DA6A42E4D5745054D0F5C244D0BBAE54C4434FF4C7A62B74DEFCBB34D300B384D8D224D4C4C3B884C0313564D986E7E4D21DDD64C04335A4DE82F994D0EB5AB4DEA82F34D6090F84D5B219B4DB5DE2F4D521AF14D14DC274E72F70B4EB98FD24DF568124E3B44224E86C5E74DEF61A24D6CFA2D4D0B6B704C857C8B4E20AA4C4F17499A4F8B528F4F255D274FD7D0974E7AE3754E374A294EDD5F8A4E2DD08C4E5572624D74E84C4D5C9E424ECDAA5F4E82690E4E133E414DC1E48C4D55A8004E940CC64D08088D4D2EF7974CE4204B4D7C667B4DA399224DD689274A60A98A4C24F6E84CEDFF694DA48DE74D1C1DE04DF1478F4D4D06764D3C0F474D0FFAEB4C4885524D7308A64D2BCF854D6498234D5E21C04C104ACE4CA0520E4DA381634DB1F65E4DE6C1D44C5B92D64B905B2D4C33EED84C7C9DD34C2BD7784CADF1104C75A2384CD6013F4CC5DE824CBA456E4C78A8764C15938F4C4703474C99EB884CDEE45B4CE503164C9FD0814BA7600D4C14F1334CC595B24CA21D914CBB9D304C04B306AB068D06B106061FA347C10DE483C253DF55C2FE684742C55179428D98E742C709EA0195C5003F010F4C96\r\n'
    tx_frame = cidXcvrUsbl.encode_frame() + '\r\n'

    print('Tx frame  : %s'%(tx_frame[:-2]))
    print('Tx Valid  : %d'%(mo.boIsMessageValid(tx_frame)))
    print('Rx frame  : %s'%(rx_frame[:-2]))
    print('Rx Valid  : %d'%(mo.boIsMessageValid(rx_frame)))
    print('Dec frame : %s'%(str(cidXcvrUsbl.decode_response(rx_frame))))
    print('UsblAzimuth          : %f'%(cidXcvrUsbl.resp_msg_signal_azimuth_i16))
    print('UsblElevation        : %f'%(cidXcvrUsbl.resp_msg_signal_elevation_i16))
    #plt.plot(cidXcvrUsbl.resp_msg_xcor_data_af)
    #plt.plot(cidXcvrUsbl.resp_msg_xcor_cross_point_u16 , cidXcvrUsbl.resp_msg_xcor_data_af[cidXcvrUsbl.resp_msg_xcor_cross_point_u16],'rX')
    #plt.plot(cidXcvrUsbl.resp_msg_xcor_detect_u16 , cidXcvrUsbl.resp_msg_xcor_data_af[cidXcvrUsbl.resp_msg_xcor_detect_u16],'gX')
    #plt.hlines(cidXcvrUsbl.resp_msg_xcor_threshold_f,0,cidXcvrUsbl.resp_msg_xcor_length_u16,'r')
    #plt.ylabel('Signal magnitude')
    #plt.xlabel('Sample Number')
    #plt.show()
