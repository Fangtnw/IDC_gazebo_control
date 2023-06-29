import sys
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import threading
from pynput import keyboard
import yaml
import ament_index_python

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        #self.publisher_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.joint_velocities = [0.0] * 7
        self.joint_positions = [0.0] * 7
        self.get_logger().info('Node created')
        self.lock = threading.Lock()
        self.load_key_map()
        # self.key_map = {
        #     'a': '+joint0',
        #     'z': '-joint0',
        #     's': '+joint1',
        #     'x': '-joint1',
        #     'd': '+joint2',
        #     'c': '-joint2',
        #     'f': '+joint3',
        #     'v': '-joint3',
        #     'g': '+joint4',
        #     'b': '-joint4',
        #     'h': '+joint5',
        #     'n': '-joint5',
        #     'j': '+joint6',
        #     'm': '-joint6',
        #     '=': 'reset',
        #     'q': 'quit'
        # }
    def load_key_map(self):
        package_share_directory = os.path.join(
        ament_index_python.get_package_share_directory('velocity_controller'))
        file_path = os.path.join(package_share_directory, 'config', 'key_config.yaml')
        with open(file_path, 'r') as file:
            try:
                key_config = yaml.safe_load(file)
                self.key_map = key_config['key_config']
            except yaml.YAMLError as e:
                self.get_logger().error(f"Error loading key map: {str(e)}")
                sys.exit(1)
        self.key_press_times = {}  # Store the start time of each key press

    def on_press(self, key):
        try:
            received_key = key.char
        except AttributeError:
            return

        if received_key in self.key_map:
            action = self.key_map[received_key]
            self.lock.acquire()
            self.set_joint_velocity(action)
            self.lock.release()

    def on_release(self, key):
        try:
            received_key = key.char
        except AttributeError:
            return

        if received_key in self.key_map:
            self.lock.acquire()
            self.set_joint_velocity('stop')
            self.key_press_times = {}
            self.lock.release()

    def set_joint_velocity(self, action):
        current_time = time.time()
        if action == '+joint0':
            self.joint_velocities[0] = self.calculate_ramped_velocity(self.joint_velocities[0], 1.0, current_time, '+joint0')
        elif action == '-joint0':
            self.joint_velocities[0] = self.calculate_ramped_velocity(self.joint_velocities[0], -1.0, current_time, '-joint0')
        elif action == '+joint1':
            self.joint_velocities[1] = self.calculate_ramped_velocity(self.joint_velocities[1], 1.0, current_time, '+joint1')
        elif action == '-joint1':
            self.joint_velocities[1] = self.calculate_ramped_velocity(self.joint_velocities[1], -1.0, current_time, '-joint1')
        elif action == '+joint2':
            self.joint_velocities[2] = self.calculate_ramped_velocity(self.joint_velocities[2], 1.0, current_time, '+joint2')
        elif action == '-joint2':
            self.joint_velocities[2] = self.calculate_ramped_velocity(self.joint_velocities[2], -1.0, current_time, '-joint2')
        elif action == '+joint3':
            self.joint_velocities[3] = self.calculate_ramped_velocity(self.joint_velocities[3], 1.0, current_time, '+joint3')
        elif action == '-joint3':
            self.joint_velocities[3] = self.calculate_ramped_velocity(self.joint_velocities[3], -1.0, current_time, '-joint3')
        elif action == '+joint4':
            self.joint_velocities[4] = self.calculate_ramped_velocity(self.joint_velocities[4], 1.0, current_time, '+joint4')
        elif action == '-joint4':
            self.joint_velocities[4] = self.calculate_ramped_velocity(self.joint_velocities[4], -1.0, current_time, '-joint4')
        elif action == '+joint5':
            self.joint_velocities[5] = self.calculate_ramped_velocity(self.joint_velocities[5], 1.0, current_time, '+joint5')
        elif action == '-joint5':
            self.joint_velocities[5] = self.calculate_ramped_velocity(self.joint_velocities[5], -1.0, current_time, '-joint5')
        elif action == '+joint6':
            self.joint_velocities[6] = self.calculate_ramped_velocity(self.joint_velocities[6], 0.05, current_time, '+joint6')
        elif action == '-joint6':
            self.joint_velocities[6] = self.calculate_ramped_velocity(self.joint_velocities[6], -0.05, current_time, '-joint6')
        elif action == 'reset':
            self.joint_velocities = [0.0] * 7
            self.joint_positions = [0.0] * 7
        elif action == 'quit':
            self.get_logger().info('Node shutting down...')
            rclpy.shutdown()
            sys.exit(0)
        elif action == 'stop':
            self.joint_velocities = [0.0] * 7

    def calculate_ramped_velocity(self, current_velocity, target_velocity, current_time, action):
        ramp_duration = 3.0  # Duration of the ramp in seconds (from 0 to target vel)
        acceleration = (target_velocity - current_velocity) / ramp_duration

        if action not in self.key_press_times:
            self.key_press_times[action] = current_time

        elapsed_time = current_time - self.key_press_times[action]

        if elapsed_time <= ramp_duration:
            return current_velocity + acceleration * elapsed_time
        else:
            return target_velocity

    def publish_commands(self):
        rate = self.create_rate(20)

        while rclpy.ok():
            self.lock.acquire()

            command_msg_vel = Float64MultiArray()
            command_msg_vel.data = self.joint_velocities[:6]

            command_msg_pos = Float64MultiArray()
            command_msg_pos.data = self.joint_positions[:6]

            command_msg_vel.layout.dim = [MultiArrayDimension(label='velocity', size=len(command_msg_vel.data))]
            command_msg_pos.layout.dim = [MultiArrayDimension(label='position', size=len(command_msg_pos.data))]

            self.publisher_vel.publish(command_msg_vel)

            self.lock.release()

            rclpy.spin_once(self)
            rate.sleep()

def main(args=None):
    rclpy.init(args=args)

    command_publisher = CommandPublisher()
    def set_non_blocking_input():
        print("Starting keyboard listener...")
        with keyboard.Listener(on_press=command_publisher.on_press, on_release=command_publisher.on_release) as listener:
            listener.join()

    thread = threading.Thread(target=set_non_blocking_input)
    thread.daemon = True
    thread.start()

    command_publisher.publish_commands()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
