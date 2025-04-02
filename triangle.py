#!/usr/bin/env python3
import threading
import time
import rclpy
from rclpy import Parameter
from interfaces.msg import QuadControlTarget
from interfaces.msg import QuadState
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovariance
import math

# The following interfaces are available from the simulator and controller to interact with the robot:
# QuadState (from simulator):
#     std_msgs/Header header
#     geometry_msgs/PoseWithCovariance pose
#     geometry_msgs/TwistWithCovariance twist
#     geometry_msgs/Accel acceleration
#     JointState joint_state
#     bool[4] foot_contact
#     float64[12] ground_contact_force # 4 times xyz
#     bool belly_contact
# QuadControlTarget (sent to controller):
#     # linear velocities
#     float64 body_x_dot
#     float64 body_y_dot
#     # height
#     float64 world_z
#     # angular velocity
#     float64 hybrid_theta_dot
#     # orientation
#     float64 pitch
#     float64 roll


class TrianglePathWalkerNode(Node):
    def __init__(self):
        super().__init__("triangle_path_walker")

        # Get State from the Simulator
        self.quad_state_sub = self.create_subscription(QuadState, "quad_state", self.quad_state_callback, 1)
        
        # Print Callback function for simple debugging
        self.print_timer = self.create_timer(2.0, self.print_callback)
        
        # Publish cmd
        self.quad_control_target_pub = self.create_publisher(QuadControlTarget, "quad_control_target", 10)
        
        self.quad_state = None

        # Check if goal is reached
        self._goal_reached = False
        # self.goal_check_timer = self.create_timer(0.1, self.goal_check_callback)


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

        # Command the Robot to Move
        self.param_client = self.create_client(SetParameters, '/mit_controller_node/set_parameters')


    def quad_state_callback(self, msg):
        self.quad_state = msg



    def print_callback(self):
        # Function for debugging and viewing outputs
        if self.quad_state is not None:
            # print(f"Quad state: {self.quad_state}")
            # print(f"Position: {self.quad_state.pose.pose.position}")
            # print(f"X Velocity: {self.quad_state.twist.twist.linear.x}")
            # print(f"Y Velocity: {self.quad_state.twist.twist.linear.y}")
            pass



    def goal_check_callback(self):
        pass



    def get_yaw_from_state(self):
        # c = PoseWithCovariance()
        # c.pose.orientation

        # 从当前四元数获取 yaw 角度
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

        # 计算两个角度之间的最小差值
        d = a - b

        while d > math.pi:
            d -= 2 * math.pi

        while d < -math.pi:
            d += 2 * math.pi

        return d



    def set_gait(self, gait: str):

        # 切换姿态
        req = SetParameters.Request()
        req.parameters = [
            Parameter(name='simple_gait_sequencer.gait', value=gait).to_parameter_msg()
        ]
        self.param_client.call_async(req)



    def send_cmd(self, v=0.0, w=0.0):
        msg = QuadControlTarget()
        msg.body_x_dot = v
        msg.body_y_dot = 0.0
        msg.hybrid_theta_dot = w
        msg.world_z = 0.30
        msg.pitch = 0.0
        msg.roll = 0.0
        self.quad_control_target_pub.publish(msg)



    def stop(self):
        self.send_cmd(0.0, 0.0)

        
        self.set_gait("STAND")

        self._stop_requested = True
        self.get_logger().info("Stopping robot")



    def move_forward(self):
        while self.quad_state is None:
            self.get_logger().info("Waiting for quad_state...")
            time.sleep(0.1)

        # 启动步态控制器
        self.set_gait("WALKING_TROT")
        
        self.get_logger().info("Starting triangle walk")
        time.sleep(1.0)


        linear_speed = 0.2  # m/s
        edge_duration = 10  # seconds (l = 2m)
        edge_lenth = 2  # m
        turn_speed = 0.4    # rad/s
        turn_angle = 2 * math.pi / 3  # 120 degree in radians

        for i in range(3):
            self.get_logger().info(f"Walking edge {i+1}")

            # start = time.time()
            # while time.time() - start < edge_duration and rclpy.ok():
            #     self.send_cmd(v = linear_speed)
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


            self.get_logger().info("Turning 120°")
            self.send_cmd(v = 0.0, w = turn_speed)
            yaw0 = self.get_yaw_from_state()

            while rclpy.ok():
                yaw_now = self.get_yaw_from_state()
                delta_yaw = abs(self.angle_diff(yaw_now, yaw0))
                if delta_yaw >= turn_angle:
                    break
                else:
                    self.send_cmd(v = 0.0, w = turn_speed)
                    time.sleep(0.1)

            self.get_logger().info("Turn complete")

        self.stop()
        self.write_theoretical_trajectory(lenth=2)


    def write_theoretical_trajectory(self, lenth=2):

        A = (0.0, 0.0)
        B = (lenth, 0.0)
        C = (lenth / 2, math.sqrt(3) * lenth / 2)

        # 插值每条边，生成平滑路径
        def interpolate(p1, p2, steps):
            return [(p1[0] + (p2[0] - p1[0]) * t / steps,
                    p1[1] + (p2[1] - p1[1]) * t / steps)
                    for t in range(steps + 1)]

        path = []
        path += interpolate(A, B, 50)
        path += interpolate(B, C, 50)
        path += interpolate(C, A, 50)

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
    walking_controller = TrianglePathWalkerNode()

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

if __name__ == "__main__":
    main()
