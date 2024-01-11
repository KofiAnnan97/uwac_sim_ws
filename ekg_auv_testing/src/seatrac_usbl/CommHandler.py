import serial
import serial.tools.list_ports as port_list
import time
import os
import sys
import codecs
from multiprocessing import Process,Lock
from threading import Thread,Lock

from MessageTypes import *

class MsgHandler:
    def __init__(self):
        self.CmdHdlrDict = {'CID_DEFAULT'              : self.__CommonHdlr,
                            'CID_SYS_ALIVE'            : self.__CommonHdlr,
                            'CID_SYS_INFO'             : self.__CommonHdlr,
                            'CID_SYS_REBOOT'           : self.__CommonHdlr,
                            'CID_SYS_ENGINEERING'      : self.__CommonHdlr,
                            'CID_PROG_INIT'            : self.__CommonHdlr,
                            'CID_PROG_BLOCK'           : self.__CommonHdlr,
                            'CID_PROG_UPDATE'          : self.__CommonHdlr,
                            'CID_STATUS'               : self.__CommonHdlr,
                            'CID_STATUS_CFG_GET'       : self.__CommonHdlr,
                            'CID_STATUS_CFG_SET'       : self.__CommonHdlr,
                            'CID_SETTINGS_GET'         : self.__CommonHdlr,
                            'CID_SETTINGS_SET'         : self.__CommonHdlr,
                            'CID_SETTINGS_LOAD'        : self.__CommonHdlr,
                            'CID_SETTINGS_SAVE'        : self.__CommonHdlr,
                            'CID_SETTINGS_RESET'       : self.__CommonHdlr,
                            'CID_CAL_ACTION'           : self.__CommonHdlr,
                            'CID_AHRS_CAL_GET'         : self.__CommonHdlr,
                            'CID_AHRS_CAL_SET'         : self.__CommonHdlr,
                            'CID_XCVR_ANALYSE'         : self.__CommonHdlr,
                            'CID_XCVR_TX_MSG'          : self.__CommonHdlr,
                            'CID_XCVR_RX_ERR'          : self.__CommonHdlr,
                            'CID_XCVR_RX_MSG'          : self.__CommonHdlr,
                            'CID_XCVR_RX_REQ'          : self.__CommonHdlr,
                            'CID_XCVR_RX_RESP'         : self.__CommonHdlr,
                            'CID_XCVR_RX_UNHANDLED'    : self.__CommonHdlr,
                            'CID_XCVR_USBL'            : self.__CommonHdlr,
                            'CID_XCVR_FIX'             : self.__CommonHdlr,
                            'CID_XCVR_STATUS'          : self.__CommonHdlr,
                            'CID_PING_SEND'            : self.__CommonHdlr,
                            'CID_PING_REQ'             : self.__CommonHdlr,
                            'CID_PING_RESP'            : self.__CommonHdlr,
                            'CID_PING_ERROR'           : self.__CommonHdlr,
                            'CID_ECHO_SEND'            : self.__CommonHdlr,
                            'CID_ECHO_REQ'             : self.__CommonHdlr,
                            'CID_ECHO_RESP'            : self.__CommonHdlr,
                            'CID_ECHO_ERROR'           : self.__CommonHdlr,
                            'CID_NAV_QUERY_SEND'       : self.__CommonHdlr,
                            'CID_NAV_QUERY_REQ'        : self.__CommonHdlr,
                            'CID_NAV_QUERY_RESP'       : self.__CommonHdlr,
                            'CID_NAV_ERROR'            : self.__CommonHdlr,
                            'CID_NAV_REF_POS_SEND'     : self.__CommonHdlr,
                            'CID_NAV_REF_POS_UPDATE'   : self.__CommonHdlr,
                            'CID_NAV_BEACON_POS_SEND'  : self.__CommonHdlr,
                            'CID_NAV_BEACON_POS_UPDATE': self.__CommonHdlr,
                            'CID_NAV_QUEUE_SET'        : self.__CommonHdlr,
                            'CID_NAV_QUEUE_CLR'        : self.__CommonHdlr,
                            'CID_NAV_QUEUE_STATUS'     : self.__CommonHdlr,
                            'CID_NAV_STATUS_SEND'      : self.__CommonHdlr,
                            'CID_NAV_STATUS_RECEIVE'   : self.__CommonHdlr,
                            'CID_DAT_SEND'             : self.__CommonHdlr,
                            'CID_DAT_RECEIVE'          : self.__CommonHdlr,
                            'CID_DAT_ERROR'            : self.__CommonHdlr,
                            'CID_DAT_QUEUE_SET'        : self.__CommonHdlr,
                            'CID_DAT_QUEUE_CLR'        : self.__CommonHdlr,
                            'CID_DAT_QUEUE_STATUS'     : self.__CommonHdlr,
                            'CID_DEX_CLOSE'            : self.__CommonHdlr,
                            'CID_DEX_DEBUG'            : self.__CommonHdlr,
                            'CID_DEX_ENQUEUE'          : self.__CommonHdlr,
                            'CID_DEX_OPEN'             : self.__CommonHdlr,
                            'CID_DEX_RESET'            : self.__CommonHdlr,
                            'CID_DEX_SEND'             : self.__CommonHdlr,
                            'CID_DEX_SOCKETS'          : self.__CommonHdlr,
                            'CID_DEX_RECEIVE'          : self.__CommonHdlr}

    def __CommonHdlr(self, in_raw_message):
        #print('Unhandled Message')
        pass


class CommHandler:

    def __init__(self, comm_port = 'COM5', baud_rate = 115200, msg_handler = MsgHandler()):
        self.ctr = 0
        self.comm_port = comm_port
        self.baud_rate = baud_rate
        self.line_ctr = 0
        self.reconnect_flg = False
        #self.lockTransmission = False Implementation to wait for acknowledge
        #self.id2Expect = '' Implementation to wait for acknowledge
        try:
            self.ser_port = serial.Serial( port = comm_port,
                                      baudrate = baud_rate,
                                      bytesize = 8,
                                      parity = 'N',
                                      stopbits = 2,
                                      timeout = 0,
                                      xonxoff = 1, #Used to avoid receiving \x00 chars when using readline function
                                      rtscts = 0)
            if self.ser_port.isOpen():
                print('Port %s is open'%(comm_port))
            else:
                print('Error trying to open port %s'%(comm_port))
        except:
            self.ser_port = 0

        self.msg_hdlr = msg_handler
        self.txBufferArray = []
        self.lockTxBuffer = Lock()
        self.rxBufferArray = []
        self.lockRxBuffer = Lock()
        self.inBuffer = ''

    def sendMessage(self,msg):
        # FIFO buffer protected to avoid race condition
        self.lockTxBuffer.acquire()
        self.txBufferArray.append(msg)
        self.lockTxBuffer.release()
        #print('= [TX] %s ='%(msg))

    def __printSerialPortList(self):
        ports = serial.tools.list_ports.comports()
        print([port.name for port in ports])

    def __isCommPortConnected(self):
        ret = False
        if os.path.islink(self.comm_port):
            current_port = os.readlink(self.comm_port).split('/')[-1]
        else:
            current_port = self.comm_port
        try:
            ports = serial.tools.list_ports.comports()
            for port in ports:
                if port.name.split('/')[-1] in current_port:
                    ret = True
        except:
            ports = ['']
        return ret

    def __reconnect(self):
        self.reconnect_flg = False
        self.ser_port.close()
        self.ser_port.open()

    def __rxProcessData(self):
        validity_str = ''
        rxBufferStr = ''
        msg_op = MsgOperation()
        # Decode the line received in Rx Buffer. Original format is byte array,
        # it is necessary to change it to string in order to make it compatible
        # with the rest of the frame processing function.
        #rxBufferStr = self.ser_port.readline().decode("ascii")
        #print(self.ser_port.in_waiting)
        try:
            tmp = self.ser_port.read(self.ser_port.inWaiting())
        except:
            tmp = ''
            #self.ser_port.close()
            #self.ser_port.open()
            #self.ser_port.reset_input_buffer()
            self.__printSerialPortList()
            print('Error while reading serial port. Input buffer will be cleared')
        try:
            if(sys.version[0]=='3'):
                self.inBuffer += codecs.decode(tmp,'UTF-8',errors='ignore')
            else:
                self.inBuffer += tmp.decode("ascii")
        except UnicodeError:
            print(tmp)
            print('UnicodeError detected, input buffer will be flushed to wait for the next valid message')
            self.inBuffer = ''
        if((self.inBuffer.find(msg_op.START_CHAR_RX) != -1) and (self.inBuffer.find('\n') != -1)):
            rxBufferStr = self.inBuffer.split('\n',1)[0]     # Take the string content until the first end of line
            self.inBuffer = self.inBuffer.split('\n',1)[1]   # Take the remainging part to form the next message
            rxBufferStr = rxBufferStr + '\n'                 # Put back the terminator
            self.line_ctr =  self.line_ctr + 1
            #print('[%4d]: %s'%(self.line_ctr, rxBufferStr))
            if(msg_op.boIsMessageValid(rxBufferStr)):
                ## Check message type
                # print('Message ID: %s'%(msg_op.getMessageId(rxBufferStr)))
                ## Print the frame. Just for debugging purposes
                #if((self.lockTransmission == True) and (rxBufferStr.find(self.id2Expect))): Implementation to wait for acknowledge
                #    self.lockTransmission = False
                    #print('Received expected Frame')
                #print('[RX][%4d]: %s'%(self.line_ctr, rxBufferStr))
                #print('appending frame %d'%(len(self.rxBufferArray)))
                self.lockRxBuffer.acquire()
                self.rxBufferArray.append(rxBufferStr)
                self.lockRxBuffer.release()
                #print('>> [RX] %s <<'%(rxBufferStr))
            #    #self.msg_hdlr.CmdHdlrDict[msg_op.getMessageId(rxBufferStr)](rxBufferStr)
            else:
                # Just print the message
                print('[%4d]: INVALID ->%s<-'%(self.line_ctr, rxBufferStr))

    def __msgDispatcher(self):
        msg_op = MsgOperation()
        # Check wether or not we have a incomming message to process
        # FIFO buffer protected to avoid race condition
        self.lockRxBuffer.acquire()
        if len(self.rxBufferArray) > 0:
            rxBufferStr = self.rxBufferArray.pop(0)
            self.lockRxBuffer.release()
            ## Check message type and call the associated callback
            cid = msg_op.getMessageId(rxBufferStr)
            self.msg_hdlr.CmdHdlrDict[cid](rxBufferStr)
        else:
            self.lockRxBuffer.release()

    def __txProcessData(self):
        #self.ctr = self.ctr + 1
        #CID_DAT_SEND(beacon_id = 'BEACON_ID1', msgtype = 'MSG_OWAY', packet_data = 'HelloWorld!!!' + str(self.ctr))
        # FIFO buffer protected to avoid race condition
        #print('__txProcessData')
        #if ((len(self.txBufferArray) > 0) and (self.lockTransmission == False)): Implementation to wait for acknowledge
        if (len(self.txBufferArray) > 0):
            self.lockTxBuffer.acquire()
            msg2Send = self.txBufferArray.pop(0)
            self.lockTxBuffer.release()
            tx_frame = msg2Send + '\r\n'
            #self.lockTransmission = True   Implementation to wait for acknowledge
            #self.id2Expect = tx_frame[1:3] Implementation to wait for acknowledge
            #print('[TX]: %s'%(tx_frame[:-2]))
            #print('WaitFor: %s'%(tx_frame[1:3]))
            # Send the frame using serial pyserial driver
            try:
                self.ser_port.write(tx_frame.encode())
            except:
                tmp = ''
                #self.ser_port.close()
                #self.ser_port.open()
                #self.ser_port.reset_output_buffer()
                self.__printSerialPortList()
                print('Error while sending data.')

    def rxTask(self):
        while True:
            try:
                if (self.__isCommPortConnected() == True):
                    if (self.reconnect_flg == False):
                        self.__rxProcessData()
                        self.__txProcessData()
                        self.__msgDispatcher()
                    else:
                        self.__reconnect()
                elif(self.reconnect_flg == False):
                    self.reconnect_flg = True
                else:
                    print('Reconnecting...')

            except KeyboardInterrupt:
                '[KeyboardInterrupt]: Process terminated'

    def txTask(self):
        while True:
            try:
                if (self.__isCommPortConnected() == True):
                    if (self.reconnect_flg == False):
                        self.__txProcessData()
                        #self.__txProcessData()
                    else:
                        self.__reconnect()
                elif(self.reconnect_flg == False):
                    self.reconnect_flg = True
                else:
                    print('Reconnecting...')
                pass
            except KeyboardInterrupt:
                '[KeyboardInterrupt]: Process terminated'

    def cmdProcessTask(self):
        while True:
            try:
                self.__msgDispatcher()
            except KeyboardInterrupt:
                '[KeyboardInterrupt]: Process terminated'

    def __startCommHandler(self):
        # Start a dameon thread to listen for incoming messages.
        #cmd_thread = Process(target=self.cmdProcessTask, args=())
        #cmd_thread = Thread(target=self.cmdProcessTask, args=())
        #cmd_thread.daemon = True
        #cmd_thread.start()

        #rx_thread = Process(target=self.rxTask, args=())
        rx_thread = Thread(target=self.rxTask, args=())
        rx_thread.daemon = True
        rx_thread.start()

        #tx_thread = Process(target=self.txTask, args=())
        #tx_thread = Thread(target=self.txTask, args=())
        #tx_thread.daemon = True
        #tx_thread.start()

    def startCommHandler(self,lock = 0):
        # Start a dameon thread to listen for incoming messages.
        self.__startCommHandler()
        # Lock the process so the child threads do not die.
        if lock == 1:
            while True:
                time.sleep(1)

if __name__ == '__main__':
    # ============================================================
    # Message handlers section
    # ============================================================

    def CidStatusHandler(in_raw_message):
        #print('CidStatusHandler: %s'%(in_raw_message[0:4]))
        cidStatus = CID_STATUS()
        cidStatus.decode_response(in_raw_message)
        # Process data to be in a more readable format
        cidStatus.resp_msg_env_supply_u16 = cidStatus.resp_msg_env_supply_u16/1000
        cidStatus.resp_msg_env_temp_i16 = cidStatus.resp_msg_env_temp_i16/10
        cidStatus.resp_msg_timestamp_u64 = cidStatus.resp_msg_timestamp_u64/1000
        cidStatus.resp_msg_att_yaw_i16 = cidStatus.resp_msg_att_yaw_i16/10
        cidStatus.resp_msg_att_pitch_i16 = cidStatus.resp_msg_att_pitch_i16/10
        cidStatus.resp_msg_att_roll_i16 = cidStatus.resp_msg_att_roll_i16/10
        # Print out data received
        print('==========')
        print(' Volt : %f v'%(cidStatus.resp_msg_env_supply_u16))
        print(' Temp : %f C'%(cidStatus.resp_msg_env_temp_i16))
        print(' Time : %f s'%(cidStatus.resp_msg_timestamp_u64))
        print(' Yaw : %f deg'%(cidStatus.resp_msg_att_yaw_i16))
        print(' Pitch: %f deg'%(cidStatus.resp_msg_att_pitch_i16))
        print(' Roll  : %f deg'%(cidStatus.resp_msg_att_roll_i16))

    def CidSendDatHandler(in_raw_message):
        print('CidSendDatHandler: %s'%(in_raw_message[0:4]))

    def CidRcvDatHandler(in_raw_message):
        print('CidRcvDatHandler: %s'%(in_raw_message[0:4]))
    # ============================================================

    '''
    COM5 - X150 - B1
    COM6 - X110 - B15
    '''
    #BEACON_PORT_DICT = {'X150':'COM5','X110':'COM6'}
    BEACON_PORT_DICT = {'X150':'/dev/ttyUSB0','X110':'/dev/ttyUSB9'}

    # Create an object of the class MsgHandler and then reassign the handler for
    # SEND_DAT message response.
    msg_hdlr = MsgHandler()
    msg_hdlr.CmdHdlrDict['CID_STATUS'] = CidStatusHandler
    msg_hdlr.CmdHdlrDict['CID_DAT_SEND'] = CidSendDatHandler
    msg_hdlr.CmdHdlrDict['CID_DAT_RECEIVE'] = CidRcvDatHandler

    # Initialize serial communication.
    comm_hdlr = CommHandler(comm_port = BEACON_PORT_DICT['X110'], msg_handler = msg_hdlr)
    comm_hdlr.startCommHandler(lock = 1)
