#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Char.h>
class KeyboardPlugin : public gazebo::ModelPlugin
{
  // Plugin logic implementation
};
void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Initialize ROS2 node
  int argc = 0;
  char** argv = nullptr;
  ros::init(argc, argv, "keyboard_plugin_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  // Create a ROS2 publisher
  ros::Publisher pub = nh.advertise<std_msgs::Char>("keyboard_input", 1);

  // Connect the keyboard input callback to publish messages
  gazebo::event::Events::ConnectKeyPress(std::bind(&KeyboardPlugin::OnKeyPress, this, std::placeholders::_1));

  // Other plugin initialization logic
}

void OnKeyPress(const unsigned char _key)
{
  // Create a ROS2 message
  std_msgs::Char msg;
  msg.data = _key;

  // Publish the message
  pub.publish(msg);
}
