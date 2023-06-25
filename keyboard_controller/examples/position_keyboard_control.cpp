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

  auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);

  RCLCPP_INFO(node->get_logger(), "Node created");

  // Set terminal attributes for non-blocking input
  setNonBlockingInput();
  
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("keyboard_controller");
  std::string file_path = package_share_directory + "/config/key_config.yaml";
    YAML::Node keyConfig = YAML::LoadFile(file_path);
  
  double joint0_position = 0.0;     // Initial position of joint0
  double joint1_position = 0.0;     // Initial position of joint1
  double joint2_position = 0.0;     // Initial position of joint2
  double joint3_position = 0.0;     // Initial position of joint3
  double joint4_position = 0.0;     // Initial position of joint4
  double joint5_position = 0.0;     // Initial position of joint5
  double joint6_position = 0.0;     // Initial position of joint6


   std::cout << "Press a key to control the robotic arm joint position, Set incremental speed, Reset position, Quit)" << std::endl;

  // Main loop
  while (rclcpp::ok()) {
    // Get keyboard input
    char c;
    if (read(STDIN_FILENO, &c, 1) == 1) {
      // Process the key and update joint positions
      std::string package_share_directory = ament_index_cpp::get_package_share_directory("keyboard_controller");
      std::string file_path = package_share_directory + "/config/key_config.yaml";
      YAML::Node keyConfig = YAML::LoadFile(file_path);
      double joint0_increment = keyConfig["increment_param"]["joint0_increment"].as<double>();
      double joint1_increment = keyConfig["increment_param"]["joint1_increment"].as<double>();
      double joint2_increment = keyConfig["increment_param"]["joint2_increment"].as<double>();
      double joint3_increment = keyConfig["increment_param"]["joint3_increment"].as<double>();
      double joint4_increment = keyConfig["increment_param"]["joint4_increment"].as<double>();
      double joint5_increment = keyConfig["increment_param"]["joint5_increment"].as<double>();
      double joint6_increment = keyConfig["increment_param"]["joint6_increment"].as<double>();

      if (keyConfig[c]) {
        std::string action = keyConfig[c].as<std::string>();

        if (action == "+joint0") {
          joint0_position += joint0_increment;
        } else if (action == "-joint0") {
          joint0_position -= joint0_increment;
        } else if (action == "+joint1") {
          joint1_position += joint1_increment;
        } else if (action == "-joint1") {
          joint1_position -= joint1_increment;
        } else if (action == "+joint2") {
          joint2_position += joint2_increment;
        } else if (action == "-joint2") {
          joint2_position -= joint2_increment;
        } else if (action == "+joint3") {
          joint3_position += joint3_increment;
        } else if (action == "-joint3") {
          joint3_position -= joint3_increment;
        } else if (action == "+joint4") {
          joint4_position += joint4_increment;
        } else if (action == "-joint4") {
          joint4_position -= joint4_increment;
        } else if (action == "+joint5") {
          joint5_position += joint5_increment;
        } else if (action == "-joint5") {
          joint5_position -= joint5_increment;
        } else if (action == "+joint6") {
          joint6_position += joint6_increment;
        } else if (action == "-joint6") {
          joint6_position -= joint6_increment;
        } else if (action == "reset") {
          joint0_position = 0;
          joint1_position = 0;
          joint2_position = 0;
          joint3_position = 0;
          joint4_position = 0;
          joint5_position = 0;
          joint6_position = 0;
        } else if (action == "quit") {
          restoreTerminalSettings();
          rclcpp::shutdown();
          return 0;
        }
      }
      
      joint0_position = std::clamp(joint0_position, -0.75, 0.75);
      joint1_position = std::clamp(joint1_position, -0.75, 0.75);
      joint2_position = std::clamp(joint2_position, -0.75, 0.75);
      joint3_position = std::clamp(joint3_position, -3.14, 3.14);
      joint4_position = std::clamp(joint4_position, -1.00, 1.00);
      joint5_position = std::clamp(joint5_position, -3.14, 3.14);
      joint6_position = std::clamp(joint6_position, -3.14, 3.14);


      // Create and populate the command message
      auto command_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
      command_msg->layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
      command_msg->layout.dim[0].stride = 1;
      command_msg->layout.dim[0].size = 6;
      command_msg->layout.dim[0].label = "";
      command_msg->layout.data_offset = 0;
      command_msg->data.push_back(joint0_position);
      command_msg->data.push_back(joint1_position);
      command_msg->data.push_back(joint2_position);
      command_msg->data.push_back(joint3_position);
      command_msg->data.push_back(joint4_position);
      command_msg->data.push_back(joint5_position);
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