#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import QuadControlTarget
from rcl_interfaces.srv import SetParameters
from rclpy import Parameter
import time

class StopRobotNode(Node):
    def __init__(self):
        super().__init__('stop_robot')
        self.cmd_pub = self.create_publisher(QuadControlTarget, 'quad_control_target', 10)
        self.param_client = self.create_client(SetParameters, '/mit_controller_node/set_parameters')
        self.timer = self.create_timer(0.5, self.send_stop_command)
        self.sent = False

    def send_stop_command(self):
        if self.sent:
            return

        self.get_logger().info("Sending STOP command...")
        cmd = QuadControlTarget()
        cmd.body_x_dot = 0.0
        cmd.body_y_dot = 0.0
        cmd.world_z = 0.30
        cmd.hybrid_theta_dot = 0.0
        cmd.pitch = 0.0
        cmd.roll = 0.0
        self.cmd_pub.publish(cmd)

        param_req = SetParameters.Request()
        param_req.parameters = [
            Parameter(name='simple_gait_sequencer.gait', value="STAND").to_parameter_msg()
        ]
        self.param_client.call_async(param_req)

        self.sent = True
        self.get_logger().info("STOPPED")
        

def main():
    rclpy.init()
    node = StopRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
