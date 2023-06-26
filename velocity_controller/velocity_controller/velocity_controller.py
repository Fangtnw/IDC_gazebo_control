import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import threading
from pynput import keyboard
import numpy as np

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
       # self.publisher_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.joint_velocities = [0.0] * 7
        self.joint_positions = [0.0] * 7
        self.get_logger().info('Node created')
        #self.key_map = np.array(['a','z'])
        self.key_map = {
            'a': '+joint0',
            'z': '-joint0',
            's': '+joint1',
            'x': '-joint1',
            'd': '+joint2',
            'c': '-joint2',
            'f': '+joint3',
            'v': '-joint3',
            'g': '+joint4',
            'b': '-joint4',
            'h': '+joint5',
            'n': '-joint5',
            'j': '+joint6',
            'm': '-joint6',
            '=': 'reset',
            'q': 'quit'
        }
        self.array = np.array(list(self.key_map.items()))
        self.lock = threading.Lock()

    def on_press(self, key):
        key = str(key)
        print('Pressed key:', key) 
        print(type(key))
        if key == 'a' : #self.key_map
            print('ok')
            action = self.key_map[key]
            self.lock.acquire()
            # print('ok')
            if action == '+joint0':
                self.joint_velocities[0] = 1.0
                print('ok2')
            elif action == '-joint0':
                self.joint_velocities[0] = -1.0
            elif action == '+joint1':
                self.joint_velocities[1] = 1.0
            elif action == '-joint1':
                self.joint_velocities[1] = -1.0
            elif action == '+joint2':
                self.joint_velocities[2] = 1.0
            elif action == '-joint2':
                self.joint_velocities[2] = -1.0
            elif action == '+joint3':
                self.joint_velocities[3] = 12.0
            elif action == '-joint3':
                self.joint_velocities[3] = -12.0
            elif action == '+joint4':
                self.joint_velocities[4] = 50.0
            elif action == '-joint4':
                self.joint_velocities[4] = -50.0
            elif action == '+joint5':
                self.joint_velocities[5] = 105.0
            elif action == '-joint5':
                self.joint_velocities[5] = -105.0
            elif action == '+joint6':
                self.joint_velocities[6] = 0.05
            elif action == '-joint6':
                self.joint_velocities[6] = -0.05
            elif action == 'reset':
                #self.joint_positions = [0.0] * 7
                self.joint_velocities = [0.0] * 7
            elif action == 'quit':
                self.lock.release()
                self.get_logger().info('Node shutting down...')
                rclpy.shutdown()
                sys.exit(0)
            self.lock.release()
        

    def publish_commands(self):
        rate = self.create_rate(20)
       
        while rclpy.ok():
            self.lock.acquire()
            # Create Float64MultiArray messages
            command_msg_vel = Float64MultiArray()
            command_msg_vel.data = self.joint_velocities[:6]

            command_msg_pos = Float64MultiArray()
            command_msg_pos.data = self.joint_positions[:6]

            # Set the size of the messages
            command_msg_vel.layout.dim = [MultiArrayDimension(label='velocity', size=len(command_msg_vel.data))]
            command_msg_pos.layout.dim = [MultiArrayDimension(label='position', size=len(command_msg_pos.data))]

            # Publish the messages
            self.publisher_vel.publish(command_msg_vel)
            #self.publisher_pos.publish(command_msg_pos)

            self.lock.release()

            rclpy.spin_once(self)
            rate.sleep()


def main(args=None):
    rclpy.init(args=args)

    command_publisher = CommandPublisher()

    def set_non_blocking_input():
        print("Starting keyboard listener...")
        listener = keyboard.Listener(on_press=command_publisher.on_press)
        listener.start()
        listener.join()

    thread = threading.Thread(target=set_non_blocking_input)
    thread.daemon = True
    thread.start()

    command_publisher.publish_commands()
    #rclpy.spin_once()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
