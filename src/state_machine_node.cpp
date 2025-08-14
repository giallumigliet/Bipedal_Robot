#include "rclcpp/rclcpp.hpp"

// Sostituisci con i tuoi tipi di messaggi
#include "Bipedal_Robot/msg/user_cmds.hpp"
#include "Bipedal_Robot/msg/imu_data.hpp"
#include "std_msgs/UInt8.hpp"
#include "Bipedal_Robot/include/constants.hpp"

class StateMachineNode : public rclcpp::Node
{
public:
  StateMachineNode() : Node("state_machine_node")
  {
    // Initializing variables
    falling = false;
    dead = false;
    online = false;
    
    // Initializing publisher
    state_publisher = this->create_publisher<std_msgs::UInt8>("robot_state", 10);

    // Initializing subscriber
    user_cmds_sub = this->create_subscription<Bipedal_Robot::msg::UserCmds>(
        "user_cmds", 10, std::bind(&StateMachineNode::user_cmds_callback, this, std::placeholders::_1));
    imu_data_sub = this->create_subscription<Bipedal_Robot::msg::ImuData>(
        "imu_data", 10, std::bind(&StateMachineNode::imu_data_callback, this, std::placeholders::_1));
    is_online_sub = this->create_subscription<std_msgs::msg::Bool>(
        "is_online", 10, std::bind(&StateMachineNode::is_online_callback, this, std::placeholders::_1));
  }

private:
  void user_cmds_callback(const Bipedal_Robot::msg::UserCmds::SharedPtr cmds_msg)
  {
      // Logica per gestire i comandi utente
      // Aggiorna lo stato interno del nodo
      // ...
  }

  void imu_data_callback(const Bipedal_Robot::msg::ImuData::SharedPtr imu_msg)
  { 
    // FALLING CHECK
    if ((std::abs(imu_msg.ax) > MAX_AX) || (std::abs(imu_msg.ay) > MAX_AY) || (std::abs(imu_msg.az) > MAX_AZ) || (std::abs(imu_msg.roll) > MAX_RECOVERY_ROLL) || (std::abs(imu_msg.pitch) > MAX_RECOVERY_PITCH)) {
      falling = true;
      RCLCPP_WARN(this->get_logger(), "WARNING! Robot is dead!!");
    }
    else falling = false;

    // FALLEN CHECK
    if ((std::abs(imu_msg.roll) > MAX_ROLL) || (std::abs(imu_msg.pitch) > MAX_PITCH)) {
      dead = true;
      RCLCPP_WARN(this->get_logger(), "WARNING! Robot is dead!!");
    }
    else dead = false;
  }

  void is_online_callback(const std_msgs::msg::Bool::SharedPtr online_msg)
  {
    // ESP32 CONNECTION CHECK
    online = online_msg;
    if (online) RCLCPP_WARN(this->get_logger(), "ERROR! ESP32 not connected!!");
  }

  // Defining publisher and subscribers as class members
  rclcpp::Publisher<std_msgs::UInt8>::SharedPtr state_publisher;
  rclcpp::Subscription<your_msgs::msg::UserCmds>::SharedPtr user_cmds_sub;
  rclcpp::Subscription<your_msgs::msg::ImuData>::SharedPtr imu_data_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_online_sub;
  bool falling, dead, online;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateMachineNode>());
  rclcpp::shutdown();
  return 0;
}

