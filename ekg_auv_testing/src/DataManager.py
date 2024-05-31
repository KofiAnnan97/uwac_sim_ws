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
from ekg_auv_testing.msg import *

from rospy_message_converter import json_message_converter

# Determines how data is sent through serial communication
class TrafficManager:
    def __init__(self):
        self.output_queue = list()
        self.priority_tbl = {
            '01': 1,
            bcn_pose: 1,
            '03': 3,
            bcn_frame: 3,
            '05': 2,
            '06': 9,
            '07': 9,
            '08': 2,
            '09': 5,
            loc: 1,
            '0B': 3
        }

        self.pq = PriorityQueue()
        self.pc = PacketCreator()

    def decode_msg(self):
        pass

    def encode_msg(self):
        pass

    def add_item(self, item):
        #packets = self.pc.create_packets(item)
        packets  = self.pc.create_data_packets(item)
        for packet in packets:
            msg = {
                'priority': packet[0],
                'data' : packet[1] 
            }
            self.pq.enqueue(msg)

    def get_output(self):
        self.output_queue = list()
        limit = 5
        if len(self.pq.queue) <= limit:
            limit = len(self.pq.queue)
        while limit > 0:
            item = self.pq.dequeue()
            self.output_queue.append(item)
            limit-=1
        return self.output_queue

    def retrieve_payloads(self):
        output_queue = self.get_output()
        for output in output_queue:
            #self.pc.add_data(output)
            self.pc.add_data_str(output)
        #payload_outputs = self.pc.retrieve_payloads()
        payload_outputs = self.pc.retrieve_data_payloads()
        return payload_outputs

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
        self.payloads = dict()
        self.rm_tbl = {
            '01': 'seatrac_pkg/bcn_pose_array',
            '02': 'seatrac_pkg/bcn_pose',
            '03': 'seatrac_pkg/bcn_frame_array',
            '04': 'seatrac_pkg/bcn_frame',
            '05': 'seatrac_pkg/bcn_remote_gps',
            '06': 'seatrac_pkg/bcn_status_array',
            '07': 'seatrac_pkg/bcn_status',
            '08': 'seatrac_pkg/head',
            '09': 'seatrac_pkg/IverOSI',
            '0A': 'seatrac_pkg/loc',
            '0B': 'ekg_auv_testing/VehiclePose'
        }

        self.priority_tbl = {
            '01': 1,
            '02': 1,
            '03': 3,
            '04': 3,
            '05': 2,
            '06': 9,
            '07': 9,
            '08': 2,
            '09': 5,
            '0A': 1,
            '0B': 3
        }

        self.p_tbl = {
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
            '$DMVPE': 3
        }

        self.inverse_rm_tbl = {
            'seatrac_pkg/bcn_pose_array': '01',
            'seatrac_pkg/bcn_pose': '02',
            'seatrac_pkg/bcn_frame_array': '03',
            'seatrac_pkg/bcn_frame': '04',
            'seatrac_pkg/bcn_remote_gps': '05',
            'seatrac_pkg/bcn_status_array': '06',
            'seatrac_pkg/bcn_status': '07',
            'seatrac_pkg/head': '08',
            'seatrac_pkg/IverOSI': '09',
            'seatrac_pkg/loc': '0A',
            'ekg_auv_testing/VehiclePose': '0B'   
        }

##########################################
# Dividing up Data Payloads using String #
##########################################
    def add_data_str(self, msg_str):
        uuid = msg_str.split(",")[1]
        if uuid not in self.payloads.keys():
            self.payloads[uuid] = list()
        self.payloads[uuid].append(msg_str)

    def create_data_str(self, msg):
        data_str = None
        if isinstance(msg, bcn_pose):
            data_str = f'$DMBPE,{msg.bid},{msg.stamp.secs},{round(msg.roll,4)},{round(msg.pitch, 4)},{round(msg.yaw,4)},{round(msg.x, 4)},{round(msg.y, 4)},{round(msg.z, 4)}'
        elif isinstance(msg, bcn_frame):
            if "," in msg.data or "\\" in msg.data:
                print("Data cannot have a comma (') or backslash (\) present.")
            else:    
                data_str = f'$DMBFE,{msg.bid},{msg.data}'
            #print(data_str)
        elif isinstance(msg, loc):
            data_str = f'$DMLOC,{round(msg.lat, 4)},{round(msg.long, 4)}'
        return data_str
    
    def from_str_to_msg(self, msg):
        try:
            data = re.split(r",", msg)
            m_id = data[0]
            r_msg = None
            
            if m_id == '$DMBPE':
                r_msg = bcn_pose()
                r_msg.bid = data[1]
                r_msg.stamp.secs = data[2]
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
            elif m_id == '$DMGPS':
                r_msg = loc()
                r_msg.lat = float(data[1])
                r_msg.long = float(data[2])
            return r_msg
        except Exception as e:
            print(e)
            return None

    def create_data_packets(self, msg):
        msg_str = self.create_data_str(msg)
        chunks = []
        data = re.split(r",", msg_str)
        m_id = data[0]
        priority = self.p_tbl[m_id]
        chunk_limit = 99

        p_id = str(uuid.uuid4())[:7]
        order = 1
        chunk_header = f'{m_id},{p_id}'
        if m_id == '$DMBPE':
            chunk_str = f'{chunk_header},{order},{data[1]},{data[2]}'
            chunks.append((priority,chunk_str))
            order += 1
            chunk_str = f'{chunk_header},{order},{data[3]},{data[4]},{data[5]}'
            chunks.append((priority,chunk_str))
            order += 1
            chunk_str = f'{chunk_header},f,{data[6]},{data[7]},{data[8]}'
            chunks.append((priority,chunk_str))
        elif m_id == '$DMBFE':
            idx_size = 25
            idx = 0
            data_length = len(data[2])
            if data_length < idx_size:
                chunk_str = f'{chunk_header},f,{data[1]},{data[2]}'
                chunks.append((priority,chunk_str))
            else:
                chunk_str = f'{chunk_header},{order},{data[1]},{data[2][:idx_size-len(data[1])]}\\'
                chunks.append((priority,chunk_str))
                idx += (idx_size - len(data[1]))
                order += 1
                while idx < data_length:
                    if (idx + idx_size) > data_length:
                        chunk_str = f'{chunk_header},f,{data[2][idx:]}'
                    else:
                        chunk_str = f'{chunk_header},{order},{data[2][idx:idx+idx_size]}\\'
                    idx += idx_size
                    order += 1
                    chunks.append((priority,chunk_str))
        elif m_id == '$DMLOC':
            chunk_str = f'{chunk_header},f,{data[1]},{data[2]}'
            chunks.append((priority,chunk_str))
        return chunks
    
    def packet_num(self, int_str):
        if int_str == 'f':
            return int_str
        elif int(int_str) < 10:
            return f"0{int_str}"
        else: 
            return int_str

    def order_data_chunks(self, chunks):
        try:
            if len(chunks) == 1:
                return chunks
            ordered_chunks = sorted(chunks, key=lambda packet: self.packet_num(re.split(r',', packet)[2]))
            return ordered_chunks
        except:
            return None

    def assemble_data_payload(self, chunks):
        try:
            ordered_chunks = self.order_data_chunks(chunks)
            m_id = ordered_chunks[0].split(",")[0] 
            msg_str = m_id
            for packet in ordered_chunks:
                data = re.sub("\A\$[A-Z]+,[a-zA-Z0-9]+,[a-z0-9]+,", ",", packet)
                msg_str += data
            msg_str = re.sub(r'\\,', '', msg_str)
            msg = self.from_str_to_msg(msg_str)
            return msg
        except Exception as e:
            print(e)
            return None

    def retrieve_data_payloads(self):
        output = []
        remaining = 0
        #for i in range(len(self.payloads)):
        while len(self.payloads) > remaining:
            p_id = list(self.payloads.keys())[0]
            preview = self.order_data_chunks(self.payloads[p_id])
            if preview[-1].split(",")[2] == 'f':
                chunks = self.payloads.pop(p_id)
                msg = self.assemble_data_payload(chunks)
                output.append(msg)
            else:
                remaining += 1
        #print(self.payloads)
        return output

########################################
# Dividing up Data Payloads using JSON #
########################################

    def add_data(self, msg):
        if 'UUID' not in msg.keys():
            p_id = None
        else:
            p_id = msg['UUID']

        if p_id not in self.payloads.keys():
            self.payloads[p_id] = list()
        self.payloads[p_id].append(msg)

    def __get_identifier(self, msg):
        id = None
        if isinstance(msg, bcn_pose):
            id = self.inverse_rm_tbl["seatrac_pkg/bcn_pose"]
        elif isinstance(msg, bcn_frame):
            id = self.inverse_rm_tbl["seatrac_pkg/bcn_frame"]
        elif isinstance(msg, loc):
            id = self.inverse_rm_tbl["seatrac_pkg/loc"]
        return id

    def create_packets(self, msg):
        chunks = []
        msg_str = json_message_converter.convert_ros_message_to_json(msg)
        msg_json = json.loads(msg_str)
        m_id = self.__get_identifier(msg)
        priority = self.priority_tbl[m_id]
        
        p_id = str(uuid.uuid4())[:7]

        if m_id == "02":    
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
            packet["order"] = 'f'
            packet['m_id'] = m_id
            chunks.append((priority, packet))

        elif m_id == "04":
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
                packet["order"] = 'f'
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
                        packet["order"] = 'f'
                    else:
                        packet["data"] = msg_json["data"][idx:idx+idx_size]
                        packet["UUID"] = p_id
                        packet["order"] = str(order)
                    packet['m_id'] = m_id
                    order +=1
                    idx += idx_size
                    chunks.append((priority, packet))

        elif m_id == "0A":
            msg_json['m_id'] = m_id
            chunks.append((priority,msg_json))
        else:
            return None
        return chunks

    def order_packets(self, chunks):
        if isinstance(chunks, dict):
            return chunks
        ordered_chunks = sorted(chunks, key=lambda packet: packet['order'])
        return ordered_chunks
    
    def assemble_payload(self, chunks):
        m_id = None
        msg = None

        chunks = self.order_packets(chunks)
        if isinstance(chunks, dict):
            m_id = chunks['m_id']
        elif len(chunks) > 0:
            m_id = chunks[0]['m_id']
        if self.rm_tbl[m_id] == 'seatrac_pkg/bcn_pose':
            msg = bcn_pose()
            msg.bid = chunks[0]["bid"]
            msg.stamp = chunks[0]["stamp"]
            msg.roll = float(chunks[1]["roll"])
            msg.pitch =  float(chunks[1]["pitch"])
            msg.yaw =  float(chunks[1]["yaw"])
            msg.x =  float(chunks[2]["x"])
            msg.y =  float(chunks[2]["y"])
            msg.z =  float(chunks[2]["z"])
        elif self.rm_tbl[m_id] == 'seatrac_pkg/bcn_frame': 
            msg = bcn_frame()
            for packet in chunks:
                if packet['order'] == '1':
                    msg.bid = packet['bid']
                    msg.data = packet['data']
                else:
                    msg.data += packet['data']
        elif self.rm_tbl[m_id] == 'seatrac_pkg/loc':
            msg = loc()
            msg.lat =  float(chunks["lat"])
            msg.long = float(chunks["long"])
        return msg
    
    """def retrieve_payloads(self):
        output = []
        for i in range(len(self.payloads)):
            output += self.retrieve_payload()
        return output

    def retrieve_payload(self):
        output = []
        #for p_id in self.payloads.keys():
        p_id = list(self.payloads.keys())[0]
        chunks = self.payloads.pop(p_id)
        if p_id == None:
            for chunk in chunks:
                msg = self.assemble_payload(chunk)
                output.append(msg)
        else:
            msg = self.assemble_payload(chunks)
            output.append(msg)
        return output"""
    
    def retrieve_payloads(self):
        output = []
        #remaining = 0
        #while len(self.payloads) > remaining:
        for i in range(len(self.payloads)):
            p_id = list(self.payloads.keys())[0]
            complete = True if p_id == None else False

            if complete == False:
                chunks = self.order_packets(self.payloads[p_id])
                complete = True if chunks[-1]['order'] == 'f' else False                
                #print(p_id, chunks[-1]['order'], complete)

            if complete == True:
                chunks = self.payloads.pop(p_id)
                if p_id is not None:
                    msg = self.assemble_payload(chunks)
                    output.append(msg)
                else:
                    for chunk in chunks:
                        msg = self.assemble_payload(chunk)
                        output.append(msg)
            #else:
            #    remaining += 1
        return output

###########    
# Testing #
###########

def GPSToLocalTest():
    gl = GPSToLocal(40.784, -73.965)
    to_c = gl.from_gps_to_cartesian(40.784, -73.965)
    print(f"Cartesian Global: {to_c}")
    to_c = gl.from_gps_to_ecef(40.784, -73.965)
    print(f"ECEF Global: {to_c}")
    # Vertical (South)
    loc_coord = gl.get_local_coord(40.784, -73.972)
    print(f"Local: {loc_coord}")
    to_c = gl.from_gps_to_cartesian(40.784, -73.972)
    print(f" Cartesian Global: {to_c}")
    to_c = gl.from_gps_to_ecef(40.784, -73.972)
    print(f"ECEF Global: {to_c}")
    # Horizontal (East)
    loc_coord = gl.get_local_coord(40.774, -73.965)
    print(f"Local: {loc_coord}")
    to_c = gl.from_gps_to_cartesian(40.774, -73.965)
    print(f"Cartesian Global: {to_c}")
    to_c = gl.from_gps_to_ecef(40.774, -73.965)
    print(f"ECEF Global: {to_c}")
    """gl = GPSToLocal(39.099912, -94.581213)
    loc_coord = gl.get_local_coord(38.627089, -90.200203)
    print(loc_coord)"""

def QueueTest():
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
    tm.add_item(pose1)
    tm.add_item(frame1)
    tm.add_item(lc1)

    for i in range(12):
        payloads = tm.retrieve_payloads()
        print(payloads)

def DataStrTest():
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

    obj = pose1
    print(asizeof.asizeof(obj))
    pc = PacketCreator()
    obj_str = pc.create_data_str(obj)
    print(f"{obj_str} | Size: {len(obj_str)}")
    #print(asizeof.asizeof(obj_str))
    
    packets = pc.create_data_packets(obj_str)
    print(packets)
    rand_idx_order = random.sample(range(len(packets)), len(packets))
    new_packets = []
    for i in range(len(rand_idx_order)):
        idx = rand_idx_order[i]
        new_packets.append(packets[idx][1])
    #print(new_packets)
    print(pc.assemble_data_payload(new_packets))

if __name__ == "__main__":
    print("++ Testing Code ++")
    #GPSToLocalTest()
    #QueueTest()
    DataManagerTest()
    #DataStrTest()

