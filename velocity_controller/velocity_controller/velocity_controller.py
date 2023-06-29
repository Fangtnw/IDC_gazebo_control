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
        self.joint_velocities = [0.0] * 6
        self.joint_positions = [0.0] * 6
        self.get_logger().info('Node created')
        self.lock = threading.Lock()
        self.load_key_map()

    def load_key_map(self):
        package_share_directory = os.path.join(
            ament_index_python.get_package_share_directory('velocity_controller'))
        file_path = os.path.join(package_share_directory, 'config', 'key_config.yaml')
        with open(file_path, 'r') as file:
            try:
                key_config = yaml.safe_load(file)
                self.key_map = key_config['key_config']
                velocity_params = key_config.get('velocity_param', {})
                self.target_velocities = {
                    'joint0': max(min(float(velocity_params['joint0_velocity']), 3), -3),
                    'joint1': max(min(float(velocity_params['joint1_velocity']), 0.5), -0.5),
                    'joint2': max(min(float(velocity_params['joint2_velocity']), 3), -3),
                    'joint3': max(min(float(velocity_params['joint3_velocity']), 3), -3),
                    'joint4': max(min(float(velocity_params['joint4_velocity']), 3), -3),
                    'joint5': max(min(float(velocity_params['joint5_velocity']), 3), -3)
                }
            except yaml.YAMLError as e:
                self.get_logger().error(f"Error loading key map: {str(e)}")
                sys.exit(1)
        self.key_press_times = {}  # Store the start time of each key press
        self.active_keys = set()  # Store the currently active keys

    def on_press(self, key):
        try:
            received_key = key.char
        except AttributeError:
            return

        if received_key in self.key_map:
            action = self.key_map[received_key]
            self.lock.acquire()
            self.active_keys.add(action)
            self.update_joint_velocities()
            self.lock.release()

    def on_release(self, key):
        try:
            received_key = key.char
        except AttributeError:
            return

        if received_key in self.key_map:
            self.lock.acquire()
            action = self.key_map[received_key]
            if action.startswith('+') or action.startswith('-'):
                joint_index = int(action[6])
                self.joint_velocities[joint_index] = 0.0
                self.active_keys.remove(action)
                self.key_press_times.pop(action, None)
            else:
                self.active_keys.remove(action)
                self.key_press_times = {}
            self.update_joint_velocities()
            self.lock.release()

        
    def update_joint_velocities(self):
        current_time = time.time()
        if not self.active_keys:
            self.joint_velocities = [0.0] * 6
        else:
            for action in self.active_keys:
                if action.startswith('+'):
                    joint_index = int(action[6])
                    self.joint_velocities[joint_index] = self.calculate_ramped_velocity(
                        float(self.target_velocities[f'joint{joint_index}']), current_time, action)
                elif action.startswith('-'):
                    joint_index = int(action[6])
                    self.joint_velocities[joint_index] = self.calculate_ramped_velocity(
                        -float(self.target_velocities[f'joint{joint_index}']), current_time, action)
                elif action == 'reset':
                    self.joint_velocities = [0.0] * 6
                    self.joint_positions = [0.0] * 6
                elif action == 'quit':
                    self.get_logger().info('Node shutting down...')
                    rclpy.shutdown()
                    sys.exit(0)


    def calculate_ramped_velocity(self, target_velocity, current_time, action):
        ramp_duration = 1.0  # Duration of the ramp in seconds (from 0 to target vel)
        acceleration = target_velocity / ramp_duration
        if action not in self.key_press_times:
            self.key_press_times[action] = current_time

        elapsed_time = current_time - self.key_press_times[action]

        if elapsed_time <= ramp_duration:
            return acceleration * elapsed_time
        else:
            return target_velocity


    def publish_commands(self):
        rate = self.create_rate(60)

        while rclpy.ok():
            self.lock.acquire()

            command_msg_vel = Float64MultiArray()
            command_msg_vel.data = self.joint_velocities[:6]

            command_msg_pos = Float64MultiArray()
            command_msg_pos.data = self.joint_positions[:6]

            command_msg_vel.layout.dim = [MultiArrayDimension(label='velocity', size=len(command_msg_vel.data))]
            #command_msg_pos.layout.dim = [MultiArrayDimension(label='position', size=len(command_msg_pos.data))]

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
