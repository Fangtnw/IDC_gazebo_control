import sys
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension,Empty
import threading
from pynput import keyboard
import yaml
import ament_index_python
from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import SpawnEntity
import xacro
import subprocess

from msg_interfaces.srv import TimeOut

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.joint_velocities = [0.0] * 6
        self.joint_positions = [0.0] * 6
        self.get_logger().info('Node created')
        self.lock = threading.Lock()
        self.load_key_map()
##----------------------------------------------------------------------------------------------------##
        self.spawn_server = self.create_service(TimeOut,"/timeout_command",self.timeout_callback)
        self.timeout_req = Empty()
        self.timeout = False

    def timeout_callback(self,request,response):
        self.timeout_req = request.timeout_command
        self.timeout = True
        self.joint_velocities = [0.0] * 6
        self.get_logger().info('Get timeout command request success!!!!')
        command_msg_vel = Float64MultiArray()
        command_msg_vel.data = self.joint_velocities
        command_msg_vel.layout.dim = [MultiArrayDimension(label='velocity', size=len(command_msg_vel.data))]
        self.publisher_vel.publish(command_msg_vel)
        return response
##----------------------------------------------------------------------------------------------------##

    def load_key_map(self):
        package_share_directory = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config')
        file_path = os.path.join(package_share_directory, 'key_config.yaml')
        with open(file_path, 'r') as file:
            try:
                key_config = yaml.safe_load(file)
                mappings = key_config.get('key_config', {})
                self.key_map = {value: key for key, value in mappings.items()}
                #self.key_map = key_config.get('key_config', {})
                velocity_params = key_config.get('velocity_param', {})
                self.target_velocities = {
                    'joint0': max(min(float(velocity_params['joint0_velocity']), 3), 0.6),
                    'joint1': max(min(float(velocity_params['joint1_velocity']), 0.7),  0.6),
                    'joint2': max(min(float(velocity_params['joint2_velocity']), 3), 0.6),
                    'joint3': max(min(float(velocity_params['joint3_velocity']), 3), 0.6),
                    'joint4': max(min(float(velocity_params['joint4_velocity']), 3), 0.6),
                    'joint5': max(min(float(velocity_params['joint5_velocity']), 3), 0.6)
                }
            except yaml.YAMLError as e:
                self.get_logger().error(f"Error loading key map: {str(e)}")
                sys.exit(1)
        self.key_press_times = {}  # Store the start time of each key press
        self.active_keys = set()  # Store the currently active keys

    def load_robot_description(self):
            pkg_name = 'robot_description'
            file_subpath = 'urdf/robot.xacro'
            file_path = os.path.join(ament_index_python.get_package_share_directory(pkg_name), file_subpath)
            if file_subpath.endswith('.xacro'):
            # Process Xacro macros and return XML content
                return xacro.process_file(file_path).toxml()
            else:
            # Read XML content directly
                with open(file_path, 'r') as file:
                    return file.read()
            
            
    def on_press(self, key):
        try:
            received_key = key.char
            received_key2 = key
            #print(received_key)
            if received_key == None:
                received_key = '5'
                #print(received_key,received_key2)
            else:
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
            if received_key == None:
                received_key = '5'
            else:
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
                    self.reset_robot()
                     #self.joint_positions = [0.0] * 6
                # elif action == 'connect':
                #     self.run_node_client()
                # elif action == 'quit':
                #     self.get_logger().info('Node shutting down...')
                #     rclpy.shutdown()
                #     sys.exit(0)


    def calculate_ramped_velocity(self, target_velocity, current_time, action):
        ramp_duration = 2  # Duration of the ramp in seconds (from 0 to target vel)
        initial_velocity = 0.4 if action.startswith('+') else -0.4
        acceleration = (target_velocity - initial_velocity) / ramp_duration
        if action not in self.key_press_times:
            self.key_press_times[action] = current_time

        elapsed_time = current_time - self.key_press_times[action]

        if elapsed_time <= ramp_duration:
            return initial_velocity + (acceleration * elapsed_time)
        else:
            return target_velocity
        
    def reset_robot(self):
        # Reset joint velocities
        #self.joint_velocities = [0.0] * 6
        # Respawn the robot entity
        try:
            delete_entity = self.create_client(DeleteEntity, '/delete_entity')
            while not delete_entity.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for /delete_entity service...')

            request = DeleteEntity.Request()
            request.name = "robot"
            future = delete_entity.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info('Robot deleted successfully')
            else:
                self.get_logger().error('Failed to delete robot')
        except Exception as e:
            self.get_logger().error(f'Error during robot deletion: {str(e)}')
            #self.get_logger().info('Robot deleted successfully')

        time.sleep(1)

        try:
            spawn_entity = self.create_client(SpawnEntity, '/spawn_entity')
            while not spawn_entity.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for /spawn_entity service...')

            request = SpawnEntity.Request()
            request.name = "robot"
            request.xml = self.load_robot_description()
            request.robot_namespace = ""
            request.reference_frame = "world"
            future = spawn_entity.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info('Robot respawned successfully')
            else:
                self.get_logger().error('Failed to respawn robot')
        except Exception as e:
           # self.get_logger().error(f'Error during robot respawn: {str(e)}')
            self.get_logger().info('Robot respawn successfully')

        try:
            controller_cmd = ['ros2', 'run', 'controller_manager', 'spawner', 'forward_velocity_controller']
            subprocess.run(controller_cmd, check=True)
        except Exception as e:
            self.get_logger().error(f'Error spawning controller: {str(e)}')

        python_executable = sys.executable
        os.execv(python_executable, [python_executable] + sys.argv)     

        #super().__init__('command_publisher')
        #self.publisher_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        #os.execv(sys.executable, ['python'] + sys.argv)
        # controller_cmd = ['ros2', 'run', 'controller_manager', 'spawner', 'forward_velocity_controller']
        # subprocess.run(controller_cmd, check=True)

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

    def run_node_client(self):
        try:
            package_share_directory = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config')
            script_path = os.path.join(package_share_directory, 'connect-server.bash')
            subprocess.run([script_path], check=True)
        except Exception as e:
            self.get_logger().error(f'Error running node client.js: {str(e)}')

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

    if command_publisher.timeout == False:
        command_publisher.publish_commands()
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()
