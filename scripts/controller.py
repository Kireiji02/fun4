#!/usr/bin/python3

from fun4.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        #----------------------------Variables----------------------------#
        
        #----------------------------Timer----------------------------#
        
        self.declare_parameter('frequency', 100.0)   
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        
        self.create_timer(1/self.frequency, self.timer_callback)
        
        #----------------------------Topic_Publishers----------------------------#
        
        #----------------------------Topic_Subscribers----------------------------#
        
        #----------------------------Service_Servers----------------------------#
        
        #----------------------------Service_Clients----------------------------#
        
    def timer_callback(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
