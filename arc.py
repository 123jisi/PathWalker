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


class WalkingController(Node):
    def __init__(self):
        super().__init__('walking_controller')
        # Basic Interfaces to the Simulator and Controller
        # Get State from the Simulator
        self.quad_state_sub = self.create_subscription(QuadState, 'quad_state', self.quad_state_callback, 10)
        self.quad_state = None

        # Print Callback function for simple debugging
        self.print_timer = self.create_timer(2.0, self.print_callback)
        self.quad_control_target_pub = self.create_publisher(QuadControlTarget, 'quad_control_target', 10)
        

        # Check if goal is reached
        self._goal_reached = False
        self.goal_check_timer = self.create_timer(0.1, self.goal_check_callback)

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

        self.check_goal_first = True
        self.goal_check = False
        self.yaw0 = 0.0
        self.yaw_total = 0.0



    def quad_state_callback(self, msg):
        self.quad_state = msg



    def get_yaw_from_state(self):
        ori = self.quad_state.pose.pose.orientation
        quat = [ori.x, ori.y, ori.z, ori.w]
        _, _, yaw = euler_from_quaternion(quat)
        return yaw



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
        
        self.quad_control_target_pub.publish(msg)



    def stop(self):
        self.get_logger().info("Stopping robot")
        self.send_cmd(v=0.0, w=0.0)
        self.set_gait("STAND")



    def print_callback(self):
        # Function for debugging and viewing outputs
        if self.quad_state is not None:
            # print(f"Quad state: {self.quad_state}")
            # print(f"Position: {self.quad_state.pose.pose.position}")
            # print(f"X Velocity: {self.quad_state.twist.twist.linear.x}")
            # print(f"Y Velocity: {self.quad_state.twist.twist.linear.y}")
            pass



    def goal_check_callback(self):
        if not self.goal_check:
            if self.check_goal_first:
                self.yaw0 = self.get_yaw_from_state()  # Initial orientation
                self.yaw_total = 0.0
                self.check_goal_first = False
                # self.get_logger().info("yaw:" + str(self.yaw0))
                
            else:
                yaw_now = self.get_yaw_from_state()  # Current orientation
                delta = abs(self.angle_diff(yaw_now, self.yaw0))
                self.yaw_total += delta
                self.yaw0 = yaw_now  # Update current orientation
                self.get_logger().info("delta:" + str(self.yaw_total))
                
                if self.yaw_total >= 2 * math.pi:
                    self.stop()
                    self.goal_check = True

        else:
            self.get_logger().info("Goal Finished!")
            self.write_theoretical_circle(radius=2)
            


    def move_forward(self):
        # while self.quad_state is None:
        #     time.sleep(0.1)
        # if self.quad_state is not None:
        #     print("Moving forward")
        #     param_req = SetParameters.Request()
        #     param_req.parameters = [
        #             Parameter(name='simple_gait_sequencer.gait', value="WALKING_TROT").to_parameter_msg()]
        #     self.param_client.call_async(param_req)
        #     time.sleep(1)
        #     cmdMsg = QuadControlTarget()
        #     cmdMsg.body_x_dot = 0.1
        #     cmdMsg.body_y_dot = 0.0
        #     cmdMsg.world_z = 0.30
        #     cmdMsg.hybrid_theta_dot = 0.0
        #     cmdMsg.pitch = 0.0
        #     cmdMsg.roll = 0.0
        #     self.quad_control_target_pub.publish(cmdMsg)
        
        while self.quad_state is None:
            time.sleep(0.1)

        if self.quad_state is not None:
            self.get_logger().info("Moving in an arc")

            # 设置步态为行走模式
            self.set_gait("WALKING_TROT")

            time.sleep(1)

            # 控制目标速度：向前 + 转向 = 弧形轨迹
            self.send_cmd(v=0.4, w=0.2)
            


            # # 自动停下：走一圈需 2π / ω 秒
            # time.sleep(2 * math.pi / cmdMsg.hybrid_theta_dot)

            # print("Done with full circle. Stopping.")
            # cmdMsg.body_x_dot = 0.0
            # cmdMsg.hybrid_theta_dot = 0.0
            # self.quad_control_target_pub.publish(cmdMsg)

            # # 切换为 STAND 姿态
            # param_req = SetParameters.Request()
            # param_req.parameters = [
            #     Parameter(name='simple_gait_sequencer.gait', value="STAND").to_parameter_msg()
            # ]
            # self.param_client.call_async(param_req)

            # rclpy.shutdown()


    def write_theoretical_circle(self, radius=1.0, num_points=200):

        center_x = 0.0
        center_y = radius

        path = []
        for i in range(num_points + 1):
            theta = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(theta)
            y = center_y + radius * math.sin(theta)
            path.append((x, y))

        with open("theoretical_log.txt", "w") as f:
            f.write("x, y\n")
            for x, y in path:
                f.write(f"{x}, {y}\n")

        self.get_logger().info(f"Theoretical trajectory written to theoretical_log.txt")



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
    walking_controller = WalkingController()
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
