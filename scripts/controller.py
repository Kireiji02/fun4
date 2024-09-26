#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64 
from sensor_msgs.msg import JointState


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        #----------------------------Variables----------------------------#
        
        self.mode = 'None'
        
        #----------------------------Timer----------------------------#
        
        self.declare_parameter('frequency', 100.0)   
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        
        self.create_timer(1/self.frequency, self.timer_callback)
        
        #----------------------------Topic_Publishers----------------------------#
        
        #----------------------------Topic_Subscribers----------------------------#
        
        self.create_subscription(Int64, '/flag_req', self.callback_keyboard_teleop, 10)

        #----------------------------Service_Servers----------------------------#
        
        #----------------------------Service_Clients----------------------------#
        
    def timer_callback(self):
        self.get_logger().info(f'{self.mode}')
    
    def callback_keyboard_teleop(self, msg):
        if (msg.data == 1):
            self.mode = 'IPK'
        elif (msg.data == 2):
            self.mode = 'Teleop'
        elif (msg.data == 3):
            self.mode = 'Auto'

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
