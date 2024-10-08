#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from example_interfaces.srv import Trigger
from fun4.srv import ModeSelector, ValueArray


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        #----------------------------Variables----------------------------#
        
        self.mode = 0
        self.value = [0.0, 0.0, 0.0]
        
        #----------------------------Timer----------------------------#
        
        self.declare_parameter('frequency', 100.0)   
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        
        self.create_timer(1/self.frequency, self.timer_callback)
        
        #----------------------------Topic_Publishers----------------------------#
        
        #----------------------------Topic_Subscribers----------------------------#
        
        self.create_subscription(Int64, '/flag_req', self.callback_keyboard_teleop, 10)

        #----------------------------Service_Servers----------------------------#
        
        self.create_service(Trigger, '/call', self.callback_mode_init)
        self.create_service(ValueArray, '/value', self.callback_value)
        
        #----------------------------Service_Clients----------------------------#
        
        self.mode_selection_client = self.create_client(ModeSelector, '/mode_select')
        
    def timer_callback(self):
        # self.get_logger().info(f'{self.mode}') # Mode selector Debug
        pass
    
    def callback_value(self, request:ValueArray.Request , response:ValueArray.Response):
        self.value[0] = request.x.data
        self.value[1] = request.y.data
        self.value[2] = request.z.data
        if 0.02**2 > (self.value[0]**2)+(self.value[1]**2)+(self.value[2]**2) > 0.52**2:
            response.confirm.data = True
        else:
            response.confirm.data = False
        return response
    
    def callback_mode_init(self, request:ModeSelector.Request , response:ModeSelector.Response):
        self.mode_call(float(self.value[0]),float(self.value[1]),float(self.value[2]), self.mode)
        return response
    
    def mode_call(self, x, y, z, m):
        while not self.mode_selection_client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for Server...')
        mode_request = ModeSelector.Request()
        mode_request.ipk_target.position.x = x
        mode_request.ipk_target.position.y = y
        mode_request.ipk_target.position.z = z
        mode_request.mode.data = m
        
        self.future = self.mode_selection_client.call_async(mode_request)
        self.future.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Result: {response.state}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            
            
    def callback_keyboard_teleop(self, msg):
        self.mode = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
