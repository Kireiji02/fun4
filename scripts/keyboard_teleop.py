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
    
    c: Mode Confirm
    
            """
        )
        if key in KEY_BINDINGS:
            self.send_flag(KEY_BINDINGS[key])
            self.latest = KEY_BINDINGS[key]
            print(f'          Selecting Mode : [ {KEY_BINDINGS[key]} ]')
        elif key == 'c' or key == 'C': 
            self.mode_init()
            print(f'          Mode Change Applied : [ {self.latest} ]')
        elif key == "\x03" :
            rclpy.shutdown()
        print(
            """

        >> Exit upon pressing any other keys <<
        

    ------------------------------------------------

        """
        )
        

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
