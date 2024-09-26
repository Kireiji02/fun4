#!/usr/bin/python3

import rclpy
import sys
import tty
import termios
from rclpy.node import Node
from std_msgs.msg import Int64 

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
        
        self.frequency = 100.0
        self.create_timer(1/self.frequency, self.timer_callback)
        
        self.send_flag_req_pub = self.create_publisher(Int64,'/flag_req',10)
        
    def send_flag(self,flag):
        msg = Int64()
        msg.data = flag
        self.send_flag_req_pub.publish(msg)
        
    def timer_callback(self):
        key = get_key(self.settings)
        if key in KEY_BINDINGS:
            self.send_flag(KEY_BINDINGS[key])
                
        elif key == "\x03" :
            rclpy.shutdown()
        

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
