#include "rclcpp/rclcpp.hpp"                        // include base ROS2 library in C++
#include "Bipedal_Robot/msg/user_commands.hpp"      // include custom message

class Joypad TeleopNode : public rclcpp::Node
{
public:
    JoypadTeleopNode() : Node("joypad_teleop_node")  // node name
    {
        user_cmds_pub_ = this->create_publisher<Bipedal_Robot::msg::user_commands>("/user_cmds",  10); // topic name and QoS depth (buffer length)
  
        // Setup joypad interface here
    }

    

private:
    rclcpp::Publisher<UserCommands::msg::UserCommands>::SharedPtr user_cmds_pub_;  // member variables

    void read_joypad()
    {
        // TODO: Read joypad
    }

    void joy_state_to_joy_cmd(sensor_msgs::msg::Joy::SharedPtr msg_joy)
    {
        // TODO: convert joypad data to command and state
    }

  //publish

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                            // node initialization
    rclcpp::spin(std::make_shared<JoypadTeleopNode>());  // maintain node active
    rclcpp::shutdown();                                  // node closing
    return 0;
}


