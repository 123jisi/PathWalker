#!/usr/bin/env python3
import math
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy import Parameter
from interfaces.msg import QuadControlTarget, QuadState
from rcl_interfaces.srv import SetParameters
from tf_transformations import euler_from_quaternion


class SquarePathWalkerNode(Node):
    def __init__(self):
        super().__init__('square_path_walker')

        self.quad_state_sub = self.create_subscription(QuadState, "quad_state", self.quad_state_callback, 10)
        self.quad_state = None

        self.cmd_pub = self.create_publisher(QuadControlTarget, "quad_control_target", 10)
        self.param_client = self.create_client(SetParameters, "/mit_controller_node/set_parameters")

        self._stop_requested = False

        # Timer Function for logging at 10 Hz/0.1s
        self.log_timer = self.create_timer(0.1, self.log_callback)
        # Log Limit: Not used at the moment.
        self.log_limit = 10
        # Log File
        self.log_file = "log.txt"
        self.log_header = False
        # Clear Log file at start: WARNING: THIS WILL CLEAR LAST EXPERIMENT DATA!!
        with open(self.log_file, "w") as log_file:
            print("Log file cleared")


    def quad_state_callback(self, msg):
        self.quad_state = msg



    def get_yaw_from_state(self):
        ori = self.quad_state.pose.pose.orientation
        quat = [ori.x, ori.y, ori.z, ori.w]
        _, _, yaw = euler_from_quaternion(quat)
        return yaw
    

    def get_pos_from_state(self):
        pos = self.quad_state.pose.pose.position
        x = pos.x
        y = pos.y
        return x, y



    def angle_diff(self, a, b):
        d = a - b

        while d > math.pi:
            d -= 2 * math.pi

        while d < -math.pi:
            d += 2 * math.pi

        return d
    


    def set_gait(self, gait: str):
        req = SetParameters.Request()
        req.parameters = [
            Parameter(name='simple_gait_sequencer.gait', value=gait).to_parameter_msg()
        ]
        self.param_client.call_async(req)



    def send_cmd(self, v = 0.0, w = 0.0):
        msg = QuadControlTarget()

        msg.body_x_dot = v
        msg.body_y_dot = 0.0
        msg.hybrid_theta_dot = w
        msg.world_z = 0.30
        msg.pitch = 0.0
        msg.roll = 0.0

        self.cmd_pub.publish(msg)



    def stop(self):
        self.get_logger().info("Stopping robot")
        self.send_cmd(v=0.0, w=0.0)
        self.set_gait("STAND")
        self._stop_requested = True



    def move_forward(self):
        while self.quad_state is None:
            self.get_logger().info("Waiting for quad_state...")
            time.sleep(0.1)

        self.get_logger().info("Starting square walk")
        self.set_gait("WALKING_TROT")
        time.sleep(1.0)

        linear_speed = 0.2      # 向前速度 m/s
        edge_duration = 10.0    # 每边走时间
        edge_lenth = 2          # m
        turn_speed = 0.4        # 转向速度 rad/s
        turn_angle = math.pi/2  # 每次转 90°

        for i in range(4):
            self.get_logger().info(f"Walking edge {i+1}")

            # start = time.time()
            # while time.time() - start < edge_duration and rclpy.ok():
            #     self.send_cmd(v=linear_speed, w=0.0)
            #     time.sleep(0.1)

            x0, y0 = self.get_pos_from_state()
            while rclpy.ok():
                x_now, y_now = self.get_pos_from_state()
                delta_dis = math.sqrt((x_now - x0) * (x_now - x0) + (y_now - y0) * (y_now - y0))
                if delta_dis >= edge_lenth:
                    break
                else:
                    self.send_cmd(v = linear_speed)
                    time.sleep(0.1)


            self.get_logger().info("Turning 90°")
            self.send_cmd(v=0.0, w=turn_speed)
            yaw0 = self.get_yaw_from_state()

            while rclpy.ok():
                yaw_now = self.get_yaw_from_state()
                delta = abs(self.angle_diff(yaw_now, yaw0))
                if delta >= turn_angle:
                    break
                else:
                    self.send_cmd(v=0.0, w=turn_speed)
                    time.sleep(0.1)

            self.get_logger().info("Turn complete")

        self.stop()
        self.write_theoretical_trajectory()


    def write_theoretical_trajectory(self, length=2.0):

        # 定义四个顶点
        A = (0.0, 0.0)
        B = (length, 0.0)
        C = (length, length)
        D = (0.0, length)

        def interpolate(p1, p2, steps):
            return [(p1[0] + (p2[0] - p1[0]) * t / steps,
                    p1[1] + (p2[1] - p1[1]) * t / steps)
                    for t in range(steps + 1)]

        path = []
        path += interpolate(A, B, 50)
        path += interpolate(B, C, 50)
        path += interpolate(C, D, 50)
        path += interpolate(D, A, 50)

        with open("theoretical_log.txt", "w") as f:
            f.write("x, y\n")
            for x, y in path:
                f.write(f"{x}, {y}\n")

        self.get_logger().info("Theoretical trajectory written to theoretical_log.txt")



    def log_callback(self):
        if self.quad_state is not None:
            if self.log_header:
                # Once we have the header, we can write the data
                with open(self.log_file, "a") as log_file:
                    log_file.write(f"{self.quad_state.pose.pose.position.x}, {self.quad_state.pose.pose.position.y}, {self.quad_state.pose.pose.position.z}, {self.quad_state.twist.twist.linear.x}, {self.quad_state.twist.twist.linear.y}, {self.quad_state.twist.twist.linear.z}, {self.quad_state.twist.twist.angular.x}, {self.quad_state.twist.twist.angular.y}, {self.quad_state.twist.twist.angular.z}, \n")
            else:
                # Make the header first
                with open(self.log_file, "a") as log_file:
                    log_file.write("x_pos, y_pos, z_pos, x_vel, y_vel, z_vel,  x_ang_vel, y_ang_vel, z_ang_vel, \n")
                self.log_header = True



def main(args=None):
    rclpy.init(args=args)
    walking_controller = SquarePathWalkerNode()

    def run_func():
        executor = rclpy.executors.MultiThreadedExecutor(1)
        executor.add_node(walking_controller)
        while rclpy.ok():
            walking_controller.get_logger().info("Running")
            executor.spin()
            return

    runner = threading.Thread(target=run_func)
    runner.start()
    walking_controller.move_forward()
    while rclpy.ok():
        time.sleep(0.1)
    walking_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
