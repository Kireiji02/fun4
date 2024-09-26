#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class PoseAnalyzerNode(Node):
    def __init__(self):
        super().__init__('pose_analyzer_node')
        
        #----------------------------Variables----------------------------#
        
        self.rand = [0.0,0.0,0.0]
        
        #----------------------------Timer----------------------------#
        
        self.declare_parameter('frequency', 10.0)   
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        
        self.create_timer(1/self.frequency, self.timer_callback)
        
        #----------------------------Topic_Publishers----------------------------#
        
        self.get_logger().info(f'{self.rand[0]}')
        
        #----------------------------Topic_Subscribers----------------------------#
        
        self.create_subscription(PoseStamped, '/target', self.callback_randomizer, 10)
        
        #----------------------------Service_Servers----------------------------#
        
        #----------------------------Service_Clients----------------------------#
        
    def timer_callback(self):
        self.get_logger().info(f'{[self.rand[0], self.rand[1], self.rand[2]]}')
        
    def callback_randomizer(self,msg):
        self.rand[0] = msg.pose.position.x
        self.rand[1] = msg.pose.position.y
        self.rand[2] = msg.pose.position.z

def main(args=None):
    rclpy.init(args=args)
    node = PoseAnalyzerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
