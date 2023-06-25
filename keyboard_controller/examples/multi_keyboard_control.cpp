#include <memory>
#include <thread>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

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

  auto publisher_vel = node->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_velocity_controller/commands", 10);
  auto publisher_pos = node->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);
  
  RCLCPP_INFO(node->get_logger(), "Node created");

  // Set terminal attributes for non-blocking input
  setNonBlockingInput();

  double joint0_position = 0.0;     // Initial position of joint0
  double joint1_position = 0.0;     // Initial position of joint1
  double joint2_position = 0.0;     // Initial position of joint2
  double joint3_position = 0.0;     // Initial position of joint3
  double joint4_position = 0.0;     // Initial position of joint4
  double joint5_position = 0.0;     // Initial position of joint5
  double joint6_position = 0.0;     // Initial position of joint6

  double joint0_velocity = 0.0;     // Initial velocity of joint0
  double joint1_velocity = 0.0;     // Initial velocity of joint1
  double joint2_velocity = 0.0;     // Initial velocity of joint2
  double joint3_velocity = 0.0;     // Initial velocity of joint3
  double joint4_velocity = 0.0;     // Initial velocity of joint4
  double joint5_velocity = 0.0;     // Initial velocity of joint5
  double joint6_velocity = 0.0;     // Initial velocity of joint6

   std::cout << "Press a key to control the robotic arm joint velocity, Set incremental speed, Reset position, Quit)" << std::endl;

  // Main loop
  while (rclcpp::ok()) {
    // Get keyboard input
    char c;
    if (read(STDIN_FILENO, &c, 1) == 1) {
      // Process the key and update joint velocity
      std::string package_share_directory = ament_index_cpp::get_package_share_directory("keyboard_controller");
      std::string file_path = package_share_directory + "/config/key_config.yaml";
      YAML::Node keyConfig = YAML::LoadFile(file_path);
      double joint0_vel = keyConfig["velocity_param"]["joint0_velocity"].as<double>();
      double joint1_vel = keyConfig["velocity_param"]["joint1_velocity"].as<double>();
      double joint2_vel = keyConfig["velocity_param"]["joint2_velocity"].as<double>();
      double joint3_vel = keyConfig["velocity_param"]["joint3_velocity"].as<double>();
      double joint4_vel = keyConfig["velocity_param"]["joint4_velocity"].as<double>();
      double joint5_vel = keyConfig["velocity_param"]["joint5_velocity"].as<double>();
      double joint6_vel = keyConfig["velocity_param"]["joint6_velocity"].as<double>();
      if (keyConfig[c]) {
        std::string action = keyConfig[c].as<std::string>();
        if (action == "+joint0") {
          joint0_velocity = joint0_vel;
        } else if (action == "-joint0") {
          joint0_velocity = -joint0_vel;
        } else if (action == "+joint1") {
          joint1_velocity = joint1_vel;
        } else if (action == "-joint1") {
          joint1_velocity = -joint1_vel;
        } else if (action == "+joint2") {
          joint2_velocity = joint2_vel;
        } else if (action == "-joint2") {
          joint2_velocity = -joint2_vel;
        } else if (action == "+joint3") {
          joint3_velocity = joint3_vel;
        } else if (action == "-joint3") {
          joint3_velocity = -joint3_vel;
        } else if (action == "+joint4") {
          joint4_velocity = joint4_vel;
        } else if (action == "-joint4") {
          joint4_velocity = -joint4_vel;
        } else if (action == "+joint5") {
          joint5_velocity = joint5_vel;
        } else if (action == "-joint5") {
          joint5_velocity = -joint5_vel;
        } else if (action == "+joint6") {
          joint6_velocity = joint6_vel;
        } else if (action == "-joint6") {
          joint6_velocity = -joint6_vel;
        } else if (action == "reset") {
          joint0_position = 0;
          joint1_position = 0;
          joint2_position = 0;
          joint3_position = 0;
          joint4_position = 0;
          joint5_position = 0;
          joint6_position = 0;
          joint0_velocity = 0;
          joint1_velocity = 0;
          joint2_velocity = 0;
          joint3_velocity = 0;
          joint4_velocity = 0;
          joint5_velocity = 0;
          joint6_velocity = 0;
        } else if (action == "quit") {
          restoreTerminalSettings();
          rclcpp::shutdown();
          return 0;
        } 
    }
      /*joint0_velocity = std::clamp(joint0_velocity, -1, 1);
      joint1_velocity = std::clamp(joint1_velocity, -1, 1);
      joint2_velocity = std::clamp(joint2_velocity, -1, 1);
      joint3_velocity = std::clamp(joint3_velocity, -1, 1);
      joint4_velocity = std::clamp(joint4_velocity, -1, 1);
      joint5_velocity = std::clamp(joint5_velocity, -1, 1);
      joint6_velocity = std::clamp(joint6_velocity, -1, 1);*/

      joint0_position = std::clamp(joint0_position, -0.65, 0.65);
      joint1_position = std::clamp(joint1_position, -3.14, 3.14);
      joint2_position = std::clamp(joint2_position, -3.14, 3.14);
      joint3_position = std::clamp(joint3_position, -3.14, 3.14);
      joint4_position = std::clamp(joint4_position, -3.14, 3.14);
      joint5_position = std::clamp(joint5_position, -3.14, 3.14);
      joint6_position = std::clamp(joint6_position, -3.14, 3.14);

      // Create and populate the command message
      auto command_msg_vel = std::make_shared<std_msgs::msg::Float64MultiArray>();
      command_msg_vel->layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
      command_msg_vel->layout.dim[0].stride = 1;
      command_msg_vel->layout.dim[0].size = 6;
      command_msg_vel->layout.dim[0].label = "";
      command_msg_vel->layout.data_offset = 0;
      command_msg_vel->data.push_back(joint0_velocity);
      command_msg_vel->data.push_back(joint1_velocity);
      command_msg_vel->data.push_back(joint2_velocity);
      command_msg_vel->data.push_back(joint3_velocity);
      command_msg_vel->data.push_back(joint4_velocity);
      command_msg_vel->data.push_back(joint5_velocity);
      //command_msg_vel->data.push_back(joint6_velocity);
      // Publish the command message
      publisher_vel->publish(*command_msg_vel);

      auto command_msg_pos = std::make_shared<std_msgs::msg::Float64MultiArray>();
      command_msg_pos->layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
      command_msg_pos->layout.dim[0].stride = 1;
      command_msg_pos->layout.dim[0].size = 6;
      command_msg_pos->layout.dim[0].label = "";
      command_msg_pos->layout.data_offset = 0;
      command_msg_pos->data.push_back(joint0_position);
      command_msg_pos->data.push_back(joint1_position);
      command_msg_pos->data.push_back(joint2_position);
      command_msg_pos->data.push_back(joint3_position);
      command_msg_pos->data.push_back(joint4_position);
      command_msg_pos->data.push_back(joint5_position);
      //command_msg_pos->data.push_back(joint6_position);
      // Publish the command message
      publisher_pos->publish(*command_msg_pos);
    }

    // Sleep for a short duration to control the publishing rate
    std::this_thread::sleep_for(50ms);
  }

  // Restore terminal attributes
  restoreTerminalSettings();
  rclcpp::shutdown();

  return 0;
}