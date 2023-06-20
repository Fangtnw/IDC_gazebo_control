#include <memory>
#include <thread>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

// Function to set terminal attributes for non-blocking input
void setNonBlockingInput()
{
  struct termios t;
  tcgetattr(STDIN_FILENO, &t);
  //t.c_lflag &= ~ICANON;
  t.c_lflag &= ~(ICANON | ECHO);  // Disable canonical mode and echoing
  tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

// Function to restore terminal attributes
void restoreTerminalSettings()
{
  struct termios t;
  tcgetattr(STDIN_FILENO, &t);
  t.c_lflag |= (ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      std::make_shared<rclcpp::Node>("command_publisher");

  auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);

  RCLCPP_INFO(node->get_logger(), "Node created");

  // Set terminal attributes for non-blocking input
  setNonBlockingInput();

  //double slider_increment = 0.02;  // Increment value for slider joint
  //double arm_increment = 0.02;     // Increment value for arm joint
  double joint_increment = 0.02;    // Increment value for camera joint

  double joint0_position = 0.0; 
  double joint1_position = 0.0;  // Initial position of slider joint
  double joint2_position = 0.0;     // Initial position of arm joint
  double joint3_position = 0.0;     // Initial position of camera joint
  double joint4_position = 0.0; 
  double joint5_position = 0.0;  // Initial position of slider joint
  double joint6_position = 0.0;     // Initial position of arm joint


   std::cout << "Press a key to control the robotic arm (q/a w/s e/d r/f to control each joint position, 1-3: Set incremental, h: Reset position, x: Quit)" << std::endl;

  // Main loop
  while (rclcpp::ok()) {
    // Get keyboard input
    char c;
    if (read(STDIN_FILENO, &c, 1) == 1) {
      // Process the key and update joint positions
      switch (c) {
        case 'q':  // Increase arm joint position
          joint0_position += joint_increment;
          break;
        case 'a':  // Decrease arm joint position
          joint0_position -= joint_increment;
          break;
        case 'w':  // Increase slider joint position
          joint1_position += joint_increment;
          break;
        case 's':  // Decrease slider joint position
          joint1_position -= joint_increment;
          break;
        case 'e':  // Increase camera joint position
          joint2_position += joint_increment;
          break;
        case 'd':  // Decrease camera joint position
          joint2_position -= joint_increment;
          break;
        case 'r':  // Increase camera joint position
          joint3_position += joint_increment;
          break;
        case 'f':  // Decrease camera joint position
          joint3_position -= joint_increment;
          break;  
        case 't':  // Increase slider joint position
          joint4_position += joint_increment;
          break;
        case 'g':  // Decrease slider joint position
          joint4_position -= joint_increment;
          break;
        case 'y':  // Increase camera joint position
          joint5_position += joint_increment;
          break;
        case 'h':  // Decrease camera joint position
          joint5_position -= joint_increment;
          break;
        case 'u':  // Increase camera joint position
          joint6_position += joint_increment;
          break;
        case 'j':  // Decrease camera joint position
          joint6_position -= joint_increment;
          break; 
          
        case '1':  // Set incremental to 0.02
          joint_increment = 0.02;
          break;
        case '2':  // Set incremental to 0.05
          joint_increment = 0.05;
          break;
        case '3':  // Set incremental to 0.12
          joint_increment = 0.12;
           break;
        case 'k':
          joint0_position = 0;
          joint1_position = 0;
          joint2_position = 0;
          joint3_position = 0;
          joint4_position = 0;
          joint5_position = 0;
          joint6_position = 0;
          break;
        case 'x':  // Quit
          restoreTerminalSettings();
          rclcpp::shutdown();
          return 0;
        default:
          continue;
      }
      joint0_position = std::clamp(joint0_position, -0.65, 0.65);
      joint1_position = std::clamp(joint1_position, -3.14, 3.14);
      joint2_position = std::clamp(joint2_position, -3.14, 3.14);
      joint3_position = std::clamp(joint3_position, -3.14, 3.14);
      joint4_position = std::clamp(joint4_position, -3.14, 3.14);
      joint5_position = std::clamp(joint5_position, -3.14, 3.14);
      joint6_position = std::clamp(joint6_position, -3.14, 3.14);


      // Create and populate the command message
      auto command_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
      command_msg->layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
      command_msg->layout.dim[0].stride = 1;
      command_msg->layout.dim[0].size = 7;
      command_msg->layout.dim[0].label = "";
      command_msg->layout.data_offset = 0;
      command_msg->data.push_back(joint0_position);
      command_msg->data.push_back(joint1_position);
      command_msg->data.push_back(joint2_position);
      command_msg->data.push_back(joint3_position);
      command_msg->data.push_back(joint4_position);
      command_msg->data.push_back(joint5_position);
      command_msg->data.push_back(joint6_position);
      // Publish the command message
      publisher->publish(*command_msg);
    }

    // Sleep for a short duration to control the publishing rate
    std::this_thread::sleep_for(50ms);
  }

  // Restore terminal attributes
  restoreTerminalSettings();
  rclcpp::shutdown();

  return 0;
}