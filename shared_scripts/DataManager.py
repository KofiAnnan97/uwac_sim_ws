#/usr/bin/env python3
from pympler import asizeof
import rospy
from datetime import datetime, timezone
import json
from math import sin, cos, atan2, sqrt, radians, degrees, asin, exp
import re
import random
import uuid

from seatrac_pkg.msg import *
#from ekg_auv_testing.msg import *
from std_msgs.msg import String

from rospy_message_converter import json_message_converter

# Determines how data is sent through serial communication
class TrafficManager:
    def __init__(self):
        self.PACKETS_SENT_LIMIT = 5
        self.output_queue = list()
        self.input_queue = dict()
        self.pq = PriorityQueue()
        self.pc = PacketCreator()

    def add_msg(self, packets):
        for packet in packets:
            msg = {
                'priority': packet[0],
                'data' : packet[1] 
            }
            self.pq.enqueue(msg)

    def add_msg_str(self, item):
        packets = self.pc.create_dynamic_packets_v2(item)
        self.add_msg(packets)

    def add_msg_json(self, item):
        packets = self.pc.create_json_packets(item)
        self.add_msg(packets)
    
    def add_str_packet(self, packet):
        p_id = packet.split(",")[1]
        if p_id not in self.input_queue.keys():
            self.input_queue[p_id] = list()
        self.input_queue[p_id].append(packet)

    def add_json_packet(self, packet):
        if 'UUID' not in packet.keys():
            p_id = None
        else:
            p_id = packet['UUID']

        if p_id not in self.input_queue.keys():
            self.input_queue[p_id] = list()
        self.input_queue[p_id].append(packet)

    def get_output(self):
        self.output_queue = list()
        limit = self.PACKETS_SENT_LIMIT
        if len(self.pq.queue) <= limit:
            limit = len(self.pq.queue)
        while limit > 0:
            item = self.pq.dequeue()
            self.output_queue.append(item)
            limit-=1
        return self.output_queue
    
    # Change to support JSON and Data Strings at the same time
    def retrieve_payloads_from_str(self):
        payloads = []
        for i in range(len(self.input_queue)):
            p_id = list(self.input_queue.keys())[0]
            packets = self.input_queue[p_id]
            msg = self.pc.retrieve_msg(packets)
            if msg is not None:
                payloads.append(msg)
                self.input_queue.pop(p_id)
        return payloads
    
    def retrieve_payloads_from_json(self):
        payloads = []
        for i in range(len(self.input_queue)):
            p_id = list(self.input_queue.keys())[0]
            packets = self.input_queue[p_id]
            msg = self.pc.from_json_to_msg(packets)
            if msg is not None:
                payloads.append(msg)
                self.input_queue.pop(p_id)
        return payloads

class PriorityQueue:
    def __init__(self):
        self.queue = []
        self.priority_vals = []
        self.obj_priority_dict = {}

    def __str__(self):
        return ' '.join([str(i) for i in self.queue])
    
    def isEmpty(self):
        return len(self.queue) == 0
    
    def enqueue(self, msg):
        self.queue.append(msg['data'])
        self.priority_vals.append(msg['priority'])
    
    def get_priority_by_object(data):
        pass

    def dequeue(self):
        try:
            hi_idx = 0
            for i in range(len(self.priority_vals)):
                if self.priority_vals[i] < self.priority_vals[hi_idx]:
                    hi_idx = i
            item = self.queue[hi_idx]
            #priority = self.priority_vals[hi_idx]
            del self.queue[hi_idx]
            del self.priority_vals[hi_idx]
            return item
        except Exception as e:
            print(e)

    def replace(self, data):
        item = self.dequeue()
        self.enqueue(data)
        """try:
            hi_idx = 0
            lo_idx = 100
            for i in range(len(self.priority_vals)):
                if self.priority_vals[i] > self.priority_vals[hi_idx]:
                    hi_idx = i
                elif self.priority_vals[i] > self.priority_vals[lo_idx]:
                    lo_idx = i
        except Exception as e:
            print(e)          """    
        return item

# Breaks down large messages to small chunks
# https://medium.com/workday-engineering/large-message-handling-with-kafka-chunking-vs-external-store-33b0fc4ccf14
class PacketCreator:
    def __init__(self):
        self.CHUNK_SIZE_LIMIT = 60 #40
        self.rm_tbl = {
            '$DMBPA': bcn_pose_array,
            '$DMBPE': bcn_pose,
            '$DMBFA': bcn_frame_array,
            '$DMBFE': bcn_frame,
            '$DMBRG': bcn_remote_gps,
            '$DMBSA': bcn_status_array,
            '$DMBSS': bcn_status,
            '$DMHED': head,
            '$DMIVO': IverOSI,
            '$DMLOC': loc,
            '$DMBGL': bcn_gps_loc
        #    '$DMVPE': VehiclePose
        }
        self.inv_rm_tbl = {
            bcn_pose_array: '$DMBPA',
            bcn_pose: '$DMBPE',
            bcn_frame_array: '$DMBFA',
            bcn_frame: '$DMBFE',
            bcn_remote_gps: '$DMBRG',
            bcn_status_array: '$DMBSA',
            bcn_status: '$DMBSS',
            head: '$DMHED',
            IverOSI: '$DMIVO',
            loc: '$DMLOC'
        #    VehiclePose: '$DMVPE'
        }
        self.priority_tbl = {
            '$DMBPA': 1,               
            '$DMBPE': 1,                
            '$DMBFA': 3,
            '$DMBFE': 3,
            '$DMBRG': 2,
            '$DMBSA': 9,
            '$DMBSS': 9,
            '$DMHED': 2,
            '$DMIVO': 5,
            '$DMLOC': 1,
            '$DMVPE': 3,
            '$DMBGL': 1
        }

    ####################
    # Helper Functions #
    ####################

    def get_packet_num(self, int_str):
        if 'f' in int_str:
            return 'f'
        elif int(int_str) < 10:
            return f"0{int_str}"
        else: 
            return int_str

    ##########################################
    # Dividing up Data Payloads using String #
    ##########################################
    """def chunk_large_str(self, data_str):
        str_chunks = []
        idx = 0
        idx_size = 20
        while (idx + idx_size) < len(data_str):
            str_chunks.append(data_str[idx:idx+idx_size])
            idx += idx_size
        str_chunks.append(data_str[idx:])
        return str_chunks

    # Create packets by data fields
    def create_dynamic_packets(self, msg):
        chunks = []
        chunk_limit = 250 #40

        msg_str = self.create_data_str(msg)
        if msg_str is not None:
            data = re.split(r",", msg_str)
            m_id = data[0]
            #print(data)
            data_str = re.sub('\A\$[A-Z]+,', "", msg_str)
            #print(data_str)
            priority = self.priority_tbl[m_id]
            p_id = str(uuid.uuid4())[:7]
            order = 1
            #reserved_space = 22
            chunk_size = 40
            #max_size = chunk_size - reserved_space
            idx = 1
            data_length = len(data_str)
            last_idx = 1
            #while idx < data_length:
            for i in range(100):
                chunk_header = f'{m_id},{p_id}'
                reserved_space = len(chunk_header)
                max_size = chunk_size - reserved_space
                #print(f"Max Size: {max_size}")
                if (idx + max_size) > data_length:
                    #print(data_str[idx:])
                    chunk_str = f'{chunk_header},f,{data_str[idx:]}'
                    chunks.append((priority,chunk_str)) 
                    break
                else:
                    idx_size = 0
                    for i in range(last_idx, len(data)):
                        #print(f"DATA: {data[i]}")
                        if len(data[i]) > max_size:
                            if idx_size > 0:
                                chunk_str = f'{chunk_header},{order},{data_str[idx-1:idx+idx_size-1]}\\'
                                chunks.append((priority,chunk_str)) 
                                order += 1
                            data_chunks = self.chunk_large_str(data[i])
                            for chunk in data_chunks:
                                chunk_str = f'{chunk_header},{order},{chunk}\\'
                                chunks.append((priority,chunk_str)) 
                                idx += len(chunk)
                                # print(order, chunk_str)
                                order +=1
                            last_idx += 1
                        elif (idx_size + len(data[i]) + 1) <= max_size:
                            idx_size += len(data[i]) + 1
                            #print(f"idx size: {idx_size}")
                            #print(i)
                        else:
                            last_idx = i
                            chunk_str = f'{chunk_header},{order},{data_str[idx-1:idx+idx_size-1]}\\'
                            chunks.append((priority,chunk_str)) 
                            #print(f"Last idx: {last_idx}")
                            break
                    idx += idx_size
                    order += 1
                    #print(order, chunk_str)
                if order > chunk_limit:
                    print("Too many chunks", order, ". Try using a smaller message.")
                    return None
        else:
            pass    
        return chunks"""

    def create_data_str(self, msg):
        data_str = None
        if isinstance(msg, bcn_pose):
            data_str = f'$DMBPE,{msg.bid},{msg.stamp.secs},{round(msg.roll,4)},{round(msg.pitch, 4)},{round(msg.yaw,4)},{round(msg.x, 4)},{round(msg.y, 4)},{round(msg.z, 4)}'
        elif isinstance(msg, bcn_frame):
            if "," in msg.data or "\\" in msg.data:
                print("Data cannot have a comma (') or backslash (\) present.")
            else:    
                data_str = f'$DMBFE,{msg.bid},{msg.data}'
        elif isinstance(msg, loc):
            data_str = f'$DMLOC,{round(msg.lat, 4)},{round(msg.long, 4)}'
        elif isinstance(msg, bcn_gps_loc):
            data_str = f'$DMBGL,{msg.bid},{msg.stamp},{round(msg.x, 4)},{round(msg.y, 4)},{round(msg.z, 4)}'
        return data_str
    
    def from_str_to_msg(self, msg):
        try:
            data = re.split(r",", msg)
            m_id = data[0]
            #print(data)
            r_msg = None
            if m_id == '$DMBPE':
                r_msg = bcn_pose()
                r_msg.bid = data[1]
                r_msg.stamp.secs = float(data[2])
                r_msg.roll = float(data[3])
                r_msg.pitch = float(data[4])
                r_msg.yaw = float(data[5])
                r_msg.x = float(data[6])
                r_msg.y = float(data[7])
                r_msg.z = float(data[8])
            elif m_id == '$DMBFE':
                r_msg = bcn_frame()
                r_msg.bid = data[1]
                r_msg.data = data[2]
            elif m_id == '$DMLOC':
                r_msg = loc()
                r_msg.lat = float(data[1])
                r_msg.long = float(data[2])
            elif m_id == '$DMBGL':
                r_msg = bcn_gps_loc()
                r_msg.bid = data[1]
                r_msg.stamp = float(data[2])
                r_msg.x = float(data[3])
                r_msg.y = float(data[4])
                r_msg.z = float(data[5])
            return r_msg
        except Exception as e:
            print(e)
            return None

    # Create packets based on string segments
    def create_dynamic_packets_v2(self, item):
        msg_str = self.create_data_str(item)
        chunks = []
        if msg_str is not None:
            data = re.split(r",", msg_str)
            m_id = data[0]
            data_str = re.sub('\A\$[A-Z]+,', "", msg_str)
            priority = self.priority_tbl[m_id]
            p_id = str(uuid.uuid4())[:7]
            order = 1
            idx = 0
            data_length = len(data_str)
            while idx < data_length:
                chunk_header = f'{m_id},{p_id},{order}'
                reserved_space = len(chunk_header)
                max_size = self.CHUNK_SIZE_LIMIT - reserved_space
                if (idx + max_size) > data_length:
                    chunk_str = f'{m_id},{p_id},{order}f,{data_str[idx:]}'
                else:
                    chunk_str = f'{chunk_header},{data_str[idx:idx+max_size-1]}\\'
                    order += 1
                idx += max_size -1
                if order > self.CHUNK_SIZE_LIMIT:
                    print("This message requires too many chunks (%f). Try using a smaller message."%(order))
                    return None
                chunks.append((priority,chunk_str))
            #print("Num of Packets Sent: ", len(chunks))
        else:
            return None    
        return chunks       

    def order_data_chunks(self, chunks):
        try:
            if len(chunks) == 1:
                return chunks
            ordered_chunks = sorted(chunks, key=lambda packet: self.get_packet_num(re.split(r',', packet)[2]))
            return ordered_chunks
        except:
            return None

    def assemble_data_payload(self, chunks):
        try:
            #ordered_chunks = self.order_data_chunks(chunks) 
            #print("Num of Packets Recieved: ", len(ordered_chunks))
            msg_str = chunks[0].split(",")[0]    # Adds Message ID to message string (m_id)
            for packet in chunks:
                msg_str += re.sub("\A\$[A-Z]+,[a-zA-Z0-9]+,[a-z0-9]+,", ",", packet)
            msg_str = re.sub(r'\\,', '', msg_str)
            msg = self.from_str_to_msg(msg_str)
            return msg
        except Exception as e:
            print(e)
            return None
    
    def retrieve_msg(self, packets):
        msg = None
        preview = self.order_data_chunks(packets)
        order = preview[-1].split(",")[2]
        num_of_packets = int(re.sub('f', '', order))
        if 'f' in order and len(preview) == num_of_packets:
            msg = self.assemble_data_payload(preview)
        return msg

    ########################################
    # Dividing up Data Payloads using JSON #
    ########################################

    def __get_identifier(self, msg):
        id = None
        if isinstance(msg, bcn_pose):
            id = self.inv_rm_tbl[bcn_pose]
        elif isinstance(msg, bcn_frame):
            id = self.inv_rm_tbl[bcn_frame]
        elif isinstance(msg, loc):
            id = self.inv_rm_tbl[loc]
        return id

    def create_json_packets(self, msg):
        chunks = []
        msg_str = json_message_converter.convert_ros_message_to_json(msg)
        msg_json = json.loads(msg_str)
        m_id = self.__get_identifier(msg)
        priority = self.priority_tbl[m_id]
        p_id = str(uuid.uuid4())[:7]

        if m_id =='$DMBPE':    
            packet = dict()
            packet["bid"] = msg_json["bid"]
            packet["stamp"] = msg_json["stamp"]
            packet["UUID"] = p_id
            packet["order"] = '1'
            packet['m_id'] = m_id
            chunks.append((priority, packet))

            packet = dict()
            packet["roll"] = msg_json["roll"]
            packet["pitch"] = msg_json["pitch"]
            packet["yaw"] = msg_json["yaw"]
            packet["UUID"] = p_id
            packet["order"] = '2'
            packet['m_id'] = m_id
            chunks.append((priority, packet))

            packet = dict()
            packet["x"] = msg_json["x"]
            packet["y"] = msg_json["y"]
            packet["z"] = msg_json["z"]
            packet["UUID"] = p_id
            packet["order"] = '3f'
            packet['m_id'] = m_id
            chunks.append((priority, packet))

        elif m_id == '$DMBFE':
            order = 1
            idx_size = 30
            idx = 0
            priority = self.priority_tbl[m_id]
            data_length = len(msg_json["data"])
            packet = dict()

            if data_length < 30:
                packet["bid"] = msg_json["bid"]
                packet["data"] = msg_json["data"]
                packet["UUID"] = p_id
                packet["order"] = f'{order}f'
                packet['m_id'] = m_id
                chunks.append((priority, packet))
            else:
                packet["bid"] = msg_json["bid"]
                packet["data"] = msg_json["data"][0:idx_size]
                packet["UUID"] = p_id
                packet["order"] = str(order)
                packet['m_id'] = m_id
                chunks.append((priority, packet))
                idx += idx_size
                order += 1
                while idx < data_length:
                    packet = dict()
                    if (idx + idx_size) >= data_length:
                        packet["data"] = msg_json["data"][idx:]
                        packet["UUID"] = p_id
                        packet["order"] = f'{order}f'
                    else:
                        packet["data"] = msg_json["data"][idx:idx+idx_size]
                        packet["UUID"] = p_id
                        packet["order"] = str(order)
                    packet['m_id'] = m_id
                    order +=1
                    idx += idx_size
                    chunks.append((priority, packet))

        elif m_id == '$DMLOC':
            msg_json['m_id'] = m_id
            msg_json['order'] = '1f'
            chunks.append((priority,msg_json))
        else:
            return None
        return chunks

    def order_json_packets(self, chunks):
        if len(chunks) == 1:
            return chunks
        else:
            ordered_chunks = sorted(chunks, key=lambda packet: self.get_packet_num(packet['order']))
            return ordered_chunks
    
    def assemble_json_payload(self, chunks):
        m_id = None
        msg = None
        #chunks = self.order_json_packets(chunks)
        if len(chunks) > 0:
            m_id = chunks[0]['m_id']
        if m_id == '$DMBPE':
            msg = bcn_pose()
            msg.bid = chunks[0]["bid"]
            msg.stamp = chunks[0]["stamp"]
            msg.roll = float(chunks[1]["roll"])
            msg.pitch =  float(chunks[1]["pitch"])
            msg.yaw =  float(chunks[1]["yaw"])
            msg.x =  float(chunks[2]["x"])
            msg.y =  float(chunks[2]["y"])
            msg.z =  float(chunks[2]["z"])
        elif m_id == '$DMBFE': 
            msg = bcn_frame()
            for packet in chunks:
                if packet['order'] == '1':
                    msg.bid = packet['bid']
                    msg.data = packet['data']
                else:
                    msg.data += packet['data']
        elif m_id == '$DMLOC':
            msg = loc()
            msg.lat =  float(chunks[0]["lat"])
            msg.long = float(chunks[0]["long"])
        return msg
    
    def from_json_to_msg(self, packets):
        msg = None
        preview = self.order_json_packets(packets)
        order = preview[-1]['order']
        num_of_packets = int(re.sub('f', '', order))
        if 'f' in order and len(preview) == num_of_packets:
            msg = self.assemble_json_payload(preview)
        return msg
    
###########    
# Testing #
###########

def QueueTest():
    print("+=======================+\n| Queue Management Test |\n+=======================+")
    pose1 = bcn_pose()
    pose1.bid="BEACONID1"
    pose1.stamp = rospy.Time(32, 4220) #datetime.now().timestamp()
    pose1.roll = 0.00000003
    pose1.pitch = -1.234559
    pose1.yaw = 0.523123
    pose1.x = 3.54
    pose1.y = 0.92
    pose1.z = -12.065

    frame1 = bcn_frame()
    frame1.bid = "BEACONID1"
    frame1.data = """
    Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ac turpis egestas sed tempus urna et pharetra. Sed enim ut sem viverra aliquet eget sit 
    amet tellus. Venenatis lectus magna fringilla urna. Ac tortor vitae purus faucibus ornare suspendisse sed nisi lacus. Nunc id cursus metus aliquam eleifend mi. Faucibus a pellentesque sit amet porttitor eget. 
    Diam sollicitudin tempor id eu nisl. Tempus imperdiet nulla malesuada pellentesque elit eget gravida cum sociis. Diam sit amet nisl suscipit adipiscing bibendum est ultricies integer.

    Sed egestas egestas fringilla phasellus. Mi eget mauris pharetra et. Hendrerit dolor magna eget est lorem. Hac habitasse platea dictumst quisque sagittis purus sit. Eleifend donec pretium vulputate sapien nec 
    sagittis. Nec ultrices dui sapien eget mi proin. Fringilla urna porttitor rhoncus dolor purus. Eu volutpat odio facilisis mauris sit amet massa vitae. Diam in arcu cursus euismod quis viverra nibh. Proin sed libero 
    enim sed. Arcu felis bibendum ut tristique et egestas quis ipsum. Praesent semper feugiat nibh sed pulvinar proin gravida hendrerit lectus."""
    #frame1.data = "Hello world."
       
    lc1 = loc()
    lc1.lat = 46.2131
    lc1.long = -24.9031

    pq = PriorityQueue()

    p_msg = {'priority': 1,
           'data': pose1}
    
    f_msg = {'priority': 3,
             'data': frame1}
    
    l_msg = {'priority': 2,
             'data': lc1}

    pq.enqueue(f_msg)
    pq.enqueue(p_msg)
    pq.enqueue(l_msg)
    pq.enqueue(l_msg)
    pq.enqueue(l_msg)
    pq.enqueue(f_msg)
    pq.enqueue(p_msg)
    pq.enqueue(l_msg)
    pq.enqueue(p_msg)
    pq.enqueue(f_msg)
    pq.enqueue(p_msg)

    while not pq.isEmpty():
        item = pq.dequeue()
        print(item)

def DataManagerTest():
    print("+======================+\n| Data Management Test |\n+======================+")
    pose1 = bcn_pose()
    pose1.bid="BEACONID1"
    pose1.stamp = rospy.Time(32, 4220) #datetime.now().timestamp()
    pose1.roll = 0.00000003
    pose1.pitch = -1.234559
    pose1.yaw = 0.523123
    pose1.x = 3.54
    pose1.y = 0.92
    pose1.z = -12.065

    frame1 = bcn_frame()
    frame1.bid = "BEACONID1"
    frame1.data = """
    Lorem ipsum dolor sit amet consectetur adipiscing elit sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ac turpis egestas sed tempus urna et pharetra. Sed enim ut sem viverra aliquet eget sit 
    amet tellus. Venenatis lectus magna fringilla urna. Ac tortor vitae purus faucibus ornare suspendisse sed nisi lacus. Nunc id cursus metus aliquam eleifend mi. Faucibus a pellentesque sit amet porttitor eget. 
    Diam sollicitudin tempor id eu nisl. Tempus imperdiet nulla malesuada pellentesque elit eget gravida cum sociis. Diam sit amet nisl suscipit adipiscing bibendum est ultricies integer.

    Sed egestas egestas fringilla phasellus. Mi eget mauris pharetra et. Hendrerit dolor magna eget est lorem. Hac habitasse platea dictumst quisque sagittis purus sit. Eleifend donec pretium vulputate sapien nec 
    sagittis. Nec ultrices dui sapien eget mi proin. Fringilla urna porttitor rhoncus dolor purus. Eu volutpat odio facilisis mauris sit amet massa vitae. Diam in arcu cursus euismod quis viverra nibh. Proin sed libero 
    enim sed. Arcu felis bibendum ut tristique et egestas quis ipsum. Praesent semper feugiat nibh sed pulvinar proin gravida hendrerit lectus."""
       
    lc1 = loc()
    lc1.lat = 46.2131
    lc1.long = -24.9031

    tm = TrafficManager()
    tm.add_msg_str(pose1)
    tm.add_msg_str(frame1)
    tm.add_msg_str(lc1)

    for i in range(14):
        outputs = tm.get_output()
        for output in outputs:
            tm.add_str_packet(output)
        payloads = tm.retrieve_payloads_from_str()
        print(payloads)

def DataManagerJSONTest():
    print("+===========================+\n| Data Management JSON Test |\n+===========================+")
    pose1 = bcn_pose()
    pose1.bid="BEACONID1"
    pose1.stamp = rospy.Time(32, 4220) #datetime.now().timestamp()
    pose1.roll = 0.00000003
    pose1.pitch = -1.234559
    pose1.yaw = 0.523123
    pose1.x = 3.54
    pose1.y = 0.92
    pose1.z = -12.065

    frame1 = bcn_frame()
    frame1.bid = "BEACONID1"
    frame1.data = """
    Lorem ipsum dolor sit amet consectetur adipiscing elit sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ac turpis egestas sed tempus urna et pharetra. Sed enim ut sem viverra aliquet eget sit 
    amet tellus. Venenatis lectus magna fringilla urna. Ac tortor vitae purus faucibus ornare suspendisse sed nisi lacus. Nunc id cursus metus aliquam eleifend mi. Faucibus a pellentesque sit amet porttitor eget. 
    Diam sollicitudin tempor id eu nisl. Tempus imperdiet nulla malesuada pellentesque elit eget gravida cum sociis. Diam sit amet nisl suscipit adipiscing bibendum est ultricies integer.

    Sed egestas egestas fringilla phasellus. Mi eget mauris pharetra et. Hendrerit dolor magna eget est lorem. Hac habitasse platea dictumst quisque sagittis purus sit. Eleifend donec pretium vulputate sapien nec 
    sagittis. Nec ultrices dui sapien eget mi proin. Fringilla urna porttitor rhoncus dolor purus. Eu volutpat odio facilisis mauris sit amet massa vitae. Diam in arcu cursus euismod quis viverra nibh. Proin sed libero 
    enim sed. Arcu felis bibendum ut tristique et egestas quis ipsum. Praesent semper feugiat nibh sed pulvinar proin gravida hendrerit lectus."""
       
    lc1 = loc()
    lc1.lat = 46.2131
    lc1.long = -24.9031

    tm = TrafficManager()
    tm.add_msg_json(pose1)
    tm.add_msg_json(frame1)
    tm.add_msg_json(lc1)

    for i in range(14):
        outputs = tm.get_output()
        for output in outputs:
            tm.add_json_packet(output)
        payloads = tm.retrieve_payloads_from_json()
        print(payloads)

def DataStrTest():
    print("+==================+\n| Data String Test |\n+==================+")
    pose1 = bcn_pose()
    pose1.bid="BEACONID1"
    pose1.stamp = rospy.Time(1717606957, 79543) #datetime.now().timestamp()
    pose1.roll = 0.03321
    pose1.pitch = -1.234559
    pose1.yaw = 0.523123
    pose1.x = 1233.58764
    pose1.y = 1240.943762
    pose1.z = -212.065

    lc1 = loc()
    lc1.lat = 46.2131
    lc1.long = -24.9031

    frame1 = bcn_frame()
    frame1.bid = "BEACONID1"
    frame1.data = """
    Lorem ipsum dolor sit amet consectetur adipiscing elit sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ac turpis egestas sed tempus urna et pharetra. Sed enim ut sem viverra aliquet eget sit 
    amet tellus. Venenatis lectus magna fringilla urna. Ac tortor vitae purus faucibus ornare suspendisse sed nisi lacus. Nunc id cursus metus aliquam eleifend mi. Faucibus a pellentesque sit amet porttitor eget. 
    Diam sollicitudin tempor id eu nisl. Tempus imperdiet nulla malesuada pellentesque elit eget gravida cum sociis. Diam sit amet nisl suscipit adipiscing bibendum est ultricies integer.

    Sed egestas egestas fringilla phasellus. Mi eget mauris pharetra et. Hendrerit dolor magna eget est lorem. Hac habitasse platea dictumst quisque sagittis purus sit. Eleifend donec pretium vulputate sapien nec 
    sagittis. Nec ultrices dui sapien eget mi proin. Fringilla urna porttitor rhoncus dolor purus. Eu volutpat odio facilisis mauris sit amet massa vitae. Diam in arcu cursus euismod quis viverra nibh. Proin sed libero 
    enim sed. Arcu felis bibendum ut tristique et egestas quis ipsum. Praesent semper feugiat nibh sed pulvinar proin gravida hendrerit lectus."""

    bg1 = bcn_gps_loc()
    bg1.bid = "BEACON_ID1"
    bg1.stamp = 1717606957.079543
    bg1.x = 1233.58764
    bg1.y = 1240.943762
    bg1.z = -212.065

    obj = frame1
    print(asizeof.asizeof(obj))
    pc = PacketCreator()
    obj_str = pc.create_data_str(obj)
    print(f"{obj_str} | Size: {len(obj_str)}")
    #print(asizeof.asizeof(obj_str))
    
    #packets = pc.create_data_packets(obj)
    packets = pc.create_dynamic_packets_v2(obj_str)
    print(packets)
    rand_idx_order = random.sample(range(len(packets)), len(packets))
    new_packets = []
    for i in range(len(rand_idx_order)):
        idx = rand_idx_order[i]
        new_packets.append(packets[idx][1])
    #print(new_packets)
    print(asizeof.asized(new_packets, detail=1).format())
    print(pc.assemble_data_payload(new_packets))

def JsonTest():
    print("+================+\n| JSON Test |\n+================+")
    pose1 = bcn_pose()
    pose1.bid="BEACONID1"
    pose1.stamp = rospy.Time(32, 4220) #datetime.now().timestamp()
    pose1.roll = 0.03321
    pose1.pitch = -1.234559
    pose1.yaw = 0.523123
    pose1.x = 1233.58764
    pose1.y = 1240.943762
    pose1.z = -212.065

    lc1 = loc()
    lc1.lat = 46.2131
    lc1.long = -24.9031

    frame1 = bcn_frame()
    frame1.bid = "BEACONID1"
    frame1.data = """
    Lorem ipsum dolor sit amet consectetur adipiscing elit sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ac turpis egestas sed tempus urna et pharetra. Sed enim ut sem viverra aliquet eget sit 
    amet tellus. Venenatis lectus magna fringilla urna. Ac tortor vitae purus faucibus ornare suspendisse sed nisi lacus. Nunc id cursus metus aliquam eleifend mi. Faucibus a pellentesque sit amet porttitor eget. 
    Diam sollicitudin tempor id eu nisl. Tempus imperdiet nulla malesuada pellentesque elit eget gravida cum sociis. Diam sit amet nisl suscipit adipiscing bibendum est ultricies integer.

    Sed egestas egestas fringilla phasellus. Mi eget mauris pharetra et. Hendrerit dolor magna eget est lorem. Hac habitasse platea dictumst quisque sagittis purus sit. Eleifend donec pretium vulputate sapien nec 
    sagittis. Nec ultrices dui sapien eget mi proin. Fringilla urna porttitor rhoncus dolor purus. Eu volutpat odio facilisis mauris sit amet massa vitae. Diam in arcu cursus euismod quis viverra nibh. Proin sed libero 
    enim sed. Arcu felis bibendum ut tristique et egestas quis ipsum. Praesent semper feugiat nibh sed pulvinar proin gravida hendrerit lectus."""

    obj = frame1
    print(obj)#, asizeof.asizeof(obj))
    pc = PacketCreator()

    packets = pc.create_packets(obj)
    rand_idx_order = random.sample(range(len(packets)), len(packets))
    new_packets = []
    for i in range(len(rand_idx_order)):
        idx = rand_idx_order[i]
        new_packets.append(packets[idx][1])
    print(new_packets)

    for packet in new_packets:
        pc.add_packet(packet)

    print(pc.retrieve_payloads_from_str())

def FrameArraySizeTest():
    pose1 = bcn_pose()
    pose1.bid="BEACON_ID1"
    pose1.stamp = rospy.Time(32, 4220) #datetime.now().timestamp()
    pose1.roll = 0
    pose1.pitch = 0
    pose1.yaw = 0
    pose1.x = 1233.58764
    pose1.y = 1240.943762
    pose1.z = -212.065
    frame_array = bcn_frame_array()
    frame_array.frame.append(pose1)
    print("Original")
    print(f"{frame_array}\nSize: {asizeof.asizeof(frame_array)}")

    data = '$DMBPE,BEACON_ID1,32,0,0,0,1233.5876,1240.9438,-212.065'

    print("\nData String in bcn_frame")
    frame_array = bcn_frame_array()
    frame = bcn_frame()
    frame.bid = "BEACON_ID1"
    frame.data = data
    frame_array.frame.append(frame)
    print(f"{frame_array}\nSize: {asizeof.asizeof(frame_array)}")

    print("\nData String in Frame Array")
    frame_array = bcn_frame_array()
    frame_array.frame = [data]
    print(f"{frame_array}\nSize: {asizeof.asizeof(frame_array)}")

    print("\nJust Data String")
    print(f"{data}\nSize: {asizeof.asizeof(data)}")

    print("\nData String as String msg")
    print(f"{String(data=data)}\nSize: {asizeof.asizeof(String(data=data))}")

    print("\nArray of Data String Chunks")
    data = [String(data="$DMBPE,a5f29db,1,BEACONID1,32,0,0"), String(data="$DMBPE,a5f29db,2,0,1233.5876,12"), String(data="$DMBPE,a5f29db,3f,0.9438,-212.065 408")]
    print(asizeof.asized(data, detail=1).format())

if __name__ == "__main__":
    print("------------------\n++ Testing Code ++\n------------------")
    #QueueTest()
    #DataStrTest()
    #JsonTest()
    #DataManagerTest()
    #DataManagerJSONTest()
    #FrameArraySizeTest()
