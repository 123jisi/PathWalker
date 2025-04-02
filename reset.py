#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.srv import ResetSimulation

class ResetSimulationClient(Node):
    def __init__(self):
        super().__init__('reset_simulation_client')
        self.cli = self.create_client(ResetSimulation, '/reset_sim')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /reset_sim service...')

        self.send_request()

    def send_request(self):
        request = ResetSimulation.Request()

        # 设置重置位置
        request.pose.position.z = 0.4
        request.pose.orientation.x = 0.0
        request.pose.orientation.y = 0.0
        request.pose.orientation.z = 0.0
        request.pose.orientation.w = 1.0

        # 设置关节角度
        request.joint_positions = [
            -0.03287414616578515, 0.7382670992579555, -1.6656333683908857,
             0.022376335027853064, 0.7301659339175386, -1.6657410165323536,
            -0.03343962015375848, 0.7507916433141004, -1.6985474687285194,
             0.02383046084651509, 0.7426327340796195, -1.6989789512741746
        ]

        self.get_logger().info("Sending reset request...")
        future = self.cli.call_async(request)
        future.add_done_callback(self.done_callback)


    def done_callback(self, future):
        try:
            result = future.result()
            self.get_logger().info("Reset simulation done.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        rclpy.shutdown()



def main():
    rclpy.init()
    node = ResetSimulationClient()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
