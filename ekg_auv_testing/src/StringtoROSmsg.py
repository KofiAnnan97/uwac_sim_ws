import rospy
import json
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rospy_message_converter import message_converter, json_message_converter
#from ekg_auv_testing.msg import bcn_frame_array, bcn_frame, bcn_pose_array, bcn_pose, \
#                               bcn_remote_gps, bcn_status_array, bcn_status, head, \
#                                IverOSI, loc
from ekg_auv_testing.msg import USBLRequestSim, USBLResponseSim

class StringtoROSmsg:
    def __init__(self):
        pass

    def str_from_rosmsg(self, msg_name, rosmsg):
        str_msg = None
        data = rosmsg
        try:
            import json
            j_data = json.dumps(data)
            str_msg = str(j_data)
        except:
            print(f"Could not create a string from this ROS message.")
        return str_msg 

    def rosmsg_from_str(self, json_str):
        msg = None
        json_obj = json.loads(json_str)
        rosmsg_name = json_obj['rosmsg_name']
        del json_obj['rosmsg_name']
        if rosmsg_name == 'bcn_frame_array':
            msg = message_converter.convert_dictionary_to_ros_message('ekg_auv_testing/bcn_frame_array', json_obj)
        elif rosmsg_name == 'bcn_frame':
            msg = message_converter.convert_dictionary_to_ros_message('ekg_auv_testing/bcn_frame', json_obj)
        elif rosmsg_name == 'bcn_pose_array':
            msg = message_converter.convert_dictionary_to_ros_message('ekg_auv_testing/bcn_pose_array', json_obj)
        elif rosmsg_name == 'bcn_pose':
            msg = message_converter.convert_dictionary_to_ros_message('ekg_auv_testing/bcn_pose', json_obj)
        elif rosmsg_name == 'bcn_remote_gps':
            msg = message_converter.convert_dictionary_to_ros_message('ekg_auv_testing/bcn_remote_gps', json_obj)
        elif rosmsg_name == 'bcn_status_array':
            msg = message_converter.convert_dictionary_to_ros_message('ekg_auv_testing/bcn_status_array', json_obj)
        elif rosmsg_name == 'bcn_status':
            msg = message_converter.convert_dictionary_to_ros_message('ekg_auv_testing/bcn_status', json_obj)
        elif rosmsg_name == 'head':
            msg = message_converter.convert_dictionary_to_ros_message('ekg_auv_testing/head', json_obj)
        elif rosmsg_name == 'IverOSI':
            msg = message_converter.convert_dictionary_to_ros_message('ekg_auv_testing/IverOSI', json_obj)
        elif rosmsg_name == 'loc':
            msg = message_converter.convert_dictionary_to_ros_message('ekg_auv_testing/loc', json_obj)
        elif rosmsg_name == 'Path':
            msg = message_converter.convert_dictionary_to_ros_message('nav_msgs/Path', json_obj)
        elif rosmsg_name == 'USBLRequestSim':
            msg = message_converter.convert_dictionary_to_ros_message('ekg_auv_testin/USBLRequestSim', json_obj)
        elif rosmsg_name == 'USBLResponseSim':
            msg = message_converter.convert_dictionary_to_ros_message('ekg_auv_testin/USBLResponseSim', json_obj)
        return msg

# FOR TESTING 
"""if __name__ == "__main__":
    smp = StringtoROSmsg()
    
    #Test for converting ros message to string
    test_msg = bcn_frame()
    test_msg.bid = "00001"
    test_msg.data = "Hello world."
    json_str = json_message_converter.convert_ros_message_to_json(test_msg)
    json_obj = json.loads(json_str)
    json_obj["rosmsg_name"] = 'bcn_frame'

    test_str = str(json.dumps(json_obj))
    new_msg = smp.rosmsg_from_str(test_str)
    print(new_msg)"""

