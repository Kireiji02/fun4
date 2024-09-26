#!/usr/bin/python3

import rclpy
import numpy as np
from math import pi
from spatialmath import SE3
from rclpy.node import Node
import roboticstoolbox as rtb
from fun4.srv import ModeSelector
from geometry_msgs.msg import PoseStamped


class PoseAnalyzerNode(Node):
    def __init__(self):
        super().__init__('pose_analyzer_node')
        
        #----------------------------Variables----------------------------#
        
        self.rand = [0.0,0.0,0.0]
        
        #----------------------------Timer----------------------------#
        
        self.declare_parameter('frequency', 100.0)   
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        
        self.create_timer(1/self.frequency, self.timer_callback)
        
        #----------------------------Topic_Publishers----------------------------#
        
        #----------------------------Topic_Subscribers----------------------------#
        
        self.create_subscription(PoseStamped, '/target', self.callback_randomizer, 10)
        
        #----------------------------Service_Servers----------------------------#
        
        self.inverse_and_mode_server = self.create_service(ModeSelector, '/mode_select', self.callback_inverse)
        
        #----------------------------Service_Clients----------------------------#
        
    def timer_callback(self):
        # self.get_logger().info(f'{[self.rand[0], self.rand[1], self.rand[2]]}') # randomizer debug
        pass
        
    def callback_inverse(self, request:ModeSelector.Request , response:ModeSelector.Response):
        
        self.get_logger().info(f'{self.rand}')
        
        robot = rtb.DHRobot(
        [   
        rtb.RevoluteMDH(a = 0,      alpha = 0,     offset = pi/2,    d = 0.2 ),
        rtb.RevoluteMDH(a = 0,      alpha = 0,     offset = pi/2,    d = 0.12),
        rtb.RevoluteMDH(a = 0.25,   alpha = 0,     offset = 0,       d = -0.1)
        ],tool = SE3.Tx(0.28),
        name = "RRR_Robot"
        )
        
        if (request.mode.data == 1):
            T_matrix = SE3(request.ipk_target.position.x,request.ipk_target.position.y,request.ipk_target.position.z)
            q = robot.ik_LM(T_matrix)
            
        elif (request.mode.data == 2):
            pass
        
        elif (request.mode.data == 3):
            T_matrix = SE3(self.rand[0],self.rand[1],self.rand[2])
            q = robot.ik_LM(T_matrix)
            
        self.get_logger().info(f'{q[0]}')
        self.get_logger().info(f'{request.mode.data}')
        self.get_logger().info(f'--------')
        
        response.state.data = True
        response.mode.data = request.mode.data
        response.joint_position.position = [q[0][0], q[0][1], q[0][2]]
        
        return response
        
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
