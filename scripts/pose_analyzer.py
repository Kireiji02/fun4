#!/usr/bin/python3

import rclpy
import numpy as np
from math import pi
from spatialmath import SE3
from rclpy.node import Node
import roboticstoolbox as rtb
from fun4.srv import ModeSelector
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

robot = rtb.DHRobot(
        [   
        rtb.RevoluteMDH(a=0.0,       alpha=0.0,     offset=pi/2,   d=0.2  ),
        rtb.RevoluteMDH(a=0.0,       alpha=pi/2,    offset=0.0,    d=0.02 ),
        rtb.RevoluteMDH(a=0.25,      alpha=0.0,     offset=0.0,    d=0.0  )
        ], tool=SE3.Tx(0.28),
        name="RRR_Robot"
    )

class PoseAnalyzerNode(Node):
    def __init__(self):
        super().__init__('pose_analyzer_node')
        
        #----------------------------Variables----------------------------#
        
        self.rand = [0.0,0.0,0.0]
        self.q_initial = np.array([0.0, 0.0, 0.0])
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.name = ["joint_1", "joint_2", "joint_3"]
        
        #----------------------------Timer----------------------------#
        
        self.declare_parameter('frequency', 100.0)   
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        
        self.create_timer(1/self.frequency, self.timer_callback)
        
        #----------------------------Topic_Publishers----------------------------#
        
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        #----------------------------Topic_Subscribers----------------------------#
        
        self.create_subscription(PoseStamped, '/target', self.callback_randomizer, 10)
        
        #----------------------------Service_Servers----------------------------#
        
        self.inverse_and_mode_server = self.create_service(ModeSelector, '/mode_select', self.callback_inverse)
        
        #----------------------------Service_Clients----------------------------#
        
    def timer_callback(self):
        
        pass
        
    def callback_inverse(self, request:ModeSelector.Request , response:ModeSelector.Response):
        # Log the requested position
        self.get_logger().info(f"Target position: {request.ipk_target.position.x, request.ipk_target.position.y, request.ipk_target.position.z}")
        
        if (request.mode.data == 1):
            # Mode 1: Levenberg-Marquardt IK
            self.target_position = np.array([request.ipk_target.position.x,request.ipk_target.position.y,request.ipk_target.position.z])
            
        elif (request.mode.data == 2):
            # Mode 2: Not yet implemented
            pass
        
        elif (request.mode.data == 3):
            # Mode 3: Use random position from subscription
            self.target_position = np.array([self.rand[0],self.rand[1],self.rand[2]])
            
        # Initialize joint angles (start at zero or previous state)
        q = self.q_initial

        # Set learning rate and tolerance
        learning_rate = 0.01
        tolerance = 1e-4
        max_iterations = 1000
        
        # Iterative IK using Jacobian
        for i in range(max_iterations):
            # Compute forward kinematics for current joint angles
            current_position = robot.fkine(q).t
            
            # Compute error between target and current position
            error = self.target_position - current_position
            
            # Log the current FK result and error
            self.get_logger().info(f"Iteration {i}, FK result: {current_position}, Error: {np.linalg.norm(error)}")

            # If error is small enough, stop
            if np.linalg.norm(error) < tolerance:
                self.get_logger().info(f"Converged after {i} iterations.")
                break
            
            # Compute the Jacobian at the current joint configuration
            J = robot.jacob0(q)[:3, :]  # Use only the first 3 rows (linear velocities)

            
            # Use damped least squares inverse for stability
            lambda_ = 0.01  # Damping factor
            J_pseudo_inv = np.linalg.inv(J.T @ J + lambda_ * np.eye(J.shape[1])) @ J.T
            
            # Update joint angles using the Jacobian
            q = q + learning_rate * J_pseudo_inv @ error
            
            self.jointstate(q)
            
        # Final joint angles after convergence
        self.q_initial = q  # Save the final joint angles
        self.get_logger().info(f"Final joint angles: {q}")

        # Send response with the computed joint positions
        response.state.data = True
        response.mode.data = request.mode.data
        response.joint_position.position = [q[0], q[1], q[2]]

        return response
    
    def jointstate(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(len(q)):
            msg.position.append(q[i])
            msg.name.append(self.name[i])
        self.joint_state_pub.publish(msg)
        
    def callback_randomizer(self, msg):
        self.rand[0] = msg.pose.position.x
        self.rand[1] = msg.pose.position.y
        self.rand[2] = msg.pose.position.z

def main(args=None):
    rclpy.init(args=args)
    node = PoseAnalyzerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
