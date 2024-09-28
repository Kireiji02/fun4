#!/usr/bin/python3

import sys
import tty
import rclpy
import termios
from rclpy.node import Node
from std_msgs.msg import Int64
from example_interfaces.srv import Trigger

# Key bindings
KEY_BINDINGS = {
    '1': 1,
    '2': 2,
    '3': 3
    }


def get_key(settings):
    """
    Capture key input from the terminal.
    """
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        self.settings = termios.tcgetattr(sys.stdin)
        
        #----------------------------Variables----------------------------#
        
        self.latest = 0
        self.mode1_value = ['','','']
        self.trig = 'neutral'
        self.value_array = [0.0,0.0,0.0]

        #----------------------------Timer----------------------------#
        
        self.declare_parameter('frequency', 100.0)   
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        
        self.create_timer(1/self.frequency, self.timer_callback)   
        
        #----------------------------Topic_Publishers----------------------------#
        
        self.send_flag_req_pub = self.create_publisher(Int64,'/flag_req',10)
        
        #----------------------------Topic_Subscribers----------------------------#
        
        #----------------------------Service_Servers----------------------------#
        
        self.mode_init_client = self.create_client(Trigger, '/call')
        
        #----------------------------Service_Clients----------------------------#
       
    def mode_init(self):
        while not self.mode_init_client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for Server...')
        mode_init_request = Trigger.Request()
        
        self.mode_init_client.call_async(mode_init_request)
        
    def send_flag(self,flag):
        msg = Int64()
        msg.data = flag
        self.send_flag_req_pub.publish(msg)
        
    def timer_callback(self):
        
        key = get_key(self.settings)

        print(
            """
            
            
    ---------------- Mode Selection ---------------- 
     
                          
                       1   2   3

    1: Inverse Pose Kinematics
    2: Teleoperation
    3: Automatic
    
            """
        )
        if key in KEY_BINDINGS and self.trig == 'neutral':
            self.send_flag(KEY_BINDINGS[key])
            self.latest = KEY_BINDINGS[key]
            print(f'          Selecting Mode : [ {KEY_BINDINGS[key]} ]')
        elif key == 'c' or key == 'C': 
            self.mode_init()
            print(f'          Mode Change Applied : [ {self.latest} ]')
        elif key == "\x03" :
            rclpy.shutdown()        
        elif key == "\x09" and self.latest == 1 and self.trig == 'neutral':
            print(f'     mode 1 input : x = []')
            self.trig = 'trig_x'
            
        elif self.latest == 1 and self.trig == 'trig_x' and (key.isnumeric() or key == '.' or key == '-' or key == "\x09"):
            if key == "\x09":
                self.trig = 'trig_y'
                print(f'     mode 1 input : x = [{self.mode1_value[0]}]\n                    y = [] (tab for next)')
                if len(self.mode1_value[0])==0:
                    self.value_array[0] = 0.0
            if key == '-':
                if len(self.mode1_value[0]) == 0:
                    self.mode1_value[0] += '-'
                    print(f'     mode 1 input : x = [{self.mode1_value[0]}] (tab for next)')
                else:
                    print(f'     mode 1 input : x = [{self.mode1_value[0]}] (tab for next)')
                    print(f'\n        >> Please input number only <<')
            if key.isnumeric():
                self.mode1_value[0] += key
                print(f'     mode 1 input : x = [{self.mode1_value[0]}] (tab for next)')
            if key == '.':
                self.trig = 'trig_x.'
                if len(self.mode1_value[0]) == 0:
                    self.mode1_value[0] += '0.'
                else:
                    self.mode1_value[0] += key
                print(f'     mode 1 input : x = [{self.mode1_value[0]}] (tab for next)')
        elif self.latest == 1 and self.trig == 'trig_x' and key.isnumeric() == False:   
            print(f'     mode 1 input : x = [{self.mode1_value[0]}]')
            print(f'\n        >> Please input number only <<')
        elif self.latest == 1 and self.trig == 'trig_x.' and (key.isnumeric() or key == "\x09"):
            if key == "\x09":
                print(f'     mode 1 input : x = [{self.mode1_value[0]}]\n                    y = [] (tab for next)')
                self.trig = 'trig_y'
                if len(self.mode1_value[0])==0:
                    self.value_array[0] = 0.0
            if key.isnumeric():
                self.mode1_value[0] += key
                print(f'     mode 1 input : x = [{self.mode1_value[0]}] (tab for next)')
        elif self.latest == 1 and self.trig == 'trig_x.' and key.isnumeric() == False:   
            print(f'     mode 1 input : x = [{self.mode1_value[0]}] (tab for next)')
            print(f'\n        >> Please input number only <<')
            
        elif self.latest == 1 and self.trig == 'trig_y' and (key.isnumeric() or key == '.' or key == '-' or key == "\x09"):
            if key == "\x09":
                self.trig = 'trig_z'
                print(f'     mode 1 input : x = [{self.mode1_value[0]}]\n                    y = [{self.mode1_value[1]}]\n                    z = [] (tab for next)')
                if len(self.mode1_value[1])==0:
                    self.value_array[1] = 0.0
            if key == '-':
                if len(self.mode1_value[1]) == 0:
                    self.mode1_value[1] += '-'
                    print(f'     mode 1 input : x = [{self.mode1_value[0]}]\n                    y = [{self.mode1_value[1]}] (tab for next)')
                else:
                    print(f'     mode 1 input : x = [{self.mode1_value[0]}]\n                    y = [{self.mode1_value[1]}] (tab for next)')
                    print(f'\n        >> Please input number only <<')
            if key.isnumeric():
                self.mode1_value[1] += key
                print(f'     mode 1 input : x = [{self.mode1_value[0]}]\n                    y = [{self.mode1_value[1]}] (tab for next)')
            if key == '.':
                self.trig = 'trig_y.'
                if len(self.mode1_value[1]) == 0:
                    self.mode1_value[1] += '0.'
                else:
                    self.mode1_value[1] += key
                print(f'     mode 1 input : x = [{self.mode1_value[0]}]\n                    y = [{self.mode1_value[1]}] (tab for next)')
        elif self.latest == 1 and self.trig == 'trig_y' and key.isnumeric() == False:   
            print(f'     mode 1 input : x = [{self.mode1_value[0]}]\n                    y = [{self.mode1_value[1]}] (tab for next)')
            print(f'\n        >> Please input number only <<')
        elif self.latest == 1 and self.trig == 'trig_y.' and (key.isnumeric() or key == "\x09"):
            if key == "\x09":
                print(f'     mode 1 input : x = [{self.mode1_value[0]}]\n                    y = [{self.mode1_value[1]}]\n                    z = [] (tab for next)')
                self.trig = 'trig_z'
                if len(self.mode1_value[1])==0:
                    self.value_array[1] = 0.0
            if key.isnumeric():
                self.mode1_value[1] += key
                print(f'     mode 1 input : x = [{self.mode1_value[0]}]\n                    y = [{self.mode1_value[1]}] (tab for next)')
        elif self.latest == 1 and self.trig == 'trig_y.' and key.isnumeric() == False:   
            print(f'     mode 1 input : x = [{self.mode1_value[0]}]\n                    y = [{self.mode1_value[1]}] (tab for next)')
            print(f'\n        >> Please input number only <<')
            
        elif self.latest == 1 and self.trig == 'trig_z' and (key.isnumeric() or key == '.' or key == '-' or key == "\x09"):
            if key == "\x09":
                self.trig = 'trig_end'
                print(f'     mode 1 input : x = [{self.mode1_value[0]}]\n                    y = [{self.mode1_value[1]}]\n                    z = [{self.mode1_value[2]}] (tab for next)')
                if len(self.mode1_value[2])==0:
                    self.value_array[2] = 0.0
            if key == '-':
                if len(self.mode1_value[2]) == 0:
                    self.mode1_value[2] += '-'
                    print(f'     mode 1 input : x = [{self.mode1_value[0]}]\n                    y = [{self.mode1_value[1]}]\n                    z = [{self.mode1_value[2]}] (tab for next)')
                else:
                    print(f'     mode 1 input : x = [{self.mode1_value[0]}]\n                    y = [{self.mode1_value[1]}]\n                    z = [{self.mode1_value[2]}] (tab for next)')
                    print(f'\n        >> Please input number only <<')
            if key.isnumeric():
                self.mode1_value[2] += key
                print(f'     mode 1 input : x = [{self.mode1_value[0]}]\n                    y = [{self.mode1_value[1]}]\n                    z = [{self.mode1_value[2]}] (tab for next)')
            if key == '.':
                self.trig = 'trig_z.'
                if len(self.mode1_value[2]) == 0:
                    self.mode1_value[2] += '0.'
                else:
                    self.mode1_value[2] += key
                print(f'     mode 1 input : x = [{self.mode1_value[0]}]\n                    y = [{self.mode1_value[1]}]\n                    z = [{self.mode1_value[2]}] (tab for next)')
        elif self.latest == 1 and self.trig == 'trig_z' and key.isnumeric() == False:   
            print(f'     mode 1 input : x = [{self.mode1_value[0]}]\n                    y = [{self.mode1_value[1]}]\n                    z = [{self.mode1_value[2]}] (tab for next)')
            print(f'\n        >> Please input number only <<')
        elif self.latest == 1 and self.trig == 'trig_z.' and (key.isnumeric() or key == "\x09"):
            if key == "\x09":
                print(f'     mode 1 input : x = [{self.mode1_value[0]}]\n                    y = [{self.mode1_value[1]}]\n                    z = [{self.mode1_value[2]}] (tab for next)')
                self.trig = 'trig_end'
                if len(self.mode1_value[2])==0:
                    self.value_array[2] = 0.0
            if key.isnumeric():
                self.mode1_value[2] += key
                print(f'     mode 1 input : x = [{self.mode1_value[0]}]\n                    y = [{self.mode1_value[1]}]\n                    z = [{self.mode1_value[2]}] (tab for next)')
        elif self.latest == 1 and self.trig == 'trig_z.' and key.isnumeric() == False:   
            print(f'     mode 1 input : x = [{self.mode1_value[0]}]\n                    y = [{self.mode1_value[1]}]\n                    z = [{self.mode1_value[2]}] (tab for next)')
            print(f'\n        >> Please input number only <<')
        elif self.latest == 1 and self.trig == 'trig_end':
            self.value_array[0] = float(self.mode1_value[0])
            self.value_array[1] = float(self.mode1_value[1])
            self.value_array[2] = float(self.mode1_value[2])
            
            print(f'     value ready : {self.value_array}, press c')
        
        if key == '1' and self.trig == 'neutral':
            print('\n        >> Press tab to input target value <<')
        
        print(
            """
        
    c: Mode Confirmation
    ------------------------------------------------"""
        )
        print(f'{key}',self.trig,self.latest,self.mode1_value,self.value_array)
        
        

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
