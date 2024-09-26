#!/usr/bin/python3

from FUN4.dummy_module import dummy_function, dummy_var
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')
        
        #----------------------------Variables----------------------------#
        
        self.r_max = 0.53
        self.r_min = 0.03
        self.z_offset = 0.2
        self.rand = [0.0,0.0,0.0]
        
        #----------------------------Timer----------------------------#
        
        self.declare_parameter('frequency', 2.0)   
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        
        self.create_timer(1/self.frequency, self.timer_callback)
        
        #----------------------------Topic_Publishers----------------------------#
        
        self.random_target_pub = self.create_publisher(PoseStamped, 'target', 10)
        
        #----------------------------Topic_Subscribers----------------------------#
        #----------------------------Service_Servers----------------------------#
        #----------------------------Service_Clients----------------------------#
        
    def timer_callback(self):
        rx = np.random.uniform(-self.r_max,self.r_max)
        ry = np.random.uniform(-self.r_max,self.r_max)
        rz = np.random.uniform(-self.r_max,self.r_max)
        
        self.rand[0] = rx if rx < -self.r_min or rx > self.r_min else np.random.uniform(-self.r_max,self.r_max)
        self.rand[1] = ry if ry < -self.r_min or ry > self.r_min else np.random.uniform(-self.r_max,self.r_max)
        self.rand[2] = rz if rz < -self.r_min or rz > self.r_min else np.random.uniform(-self.r_max,self.r_max)
        
        if (self.r_min**2 < (self.rand[0]**2 + self.rand[1]**2 + self.rand[2]**2) < self.r_max**2):
            self.random(self.rand[0], self.rand[1], self.rand[2] + self.z_offset)
            self.get_logger().info(f'{[self.rand[0], self.rand[1], self.rand[2]]}')
        else:
            self.get_logger().info('out of bounds')

        
    def random(self, x, y, z):
        msg = PoseStamped()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        self.random_target_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
