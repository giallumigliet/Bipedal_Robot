#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "bipedal_robot/msg/feetpositions.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "bipedal_robot/srv/foottrajectory.hpp" 
#include "bipedal_robot/include/constants.hpp" 

using namespace std::chrono_literals;

class TrajectoryExecutorNode : public rclcpp::Node
{
public:
  TrajectoryExecutorNode(): Node("trajectory_executor_node") {
    
    // Publisher
    feet_pub = this->create_publisher<bipedal_robot::msg::FeetPositions>("/feet_positions", 10);
    // Service client
    client = this->create_client<bipedal_robot::srv::FootTrajectory>("/trajectory_planner_node/foot_trajectory");
    // Timer → simula l’avanzamento della progress bar
    timer = this->create_wall_timer(100ms, std::bind(&TrajectoryExecutorNode::update, this));

    // Initialization, first trajectories request
    requestTrajectory(LEFT_FOOT); 
    requestTrajectory(RIGHT_FOOT); 
  }

private:
  void update() {
    // update() not extecuted until both initial trajectories are loaded
    if (initialization) {
      return;
    }
    
    if (left_index > TRAJECTORY_SIZE) {
      left_index = 0;
      left_points = next_left_points;
      left_ready = false;
    }
    if (right_index > TRAJECTORY_SIZE) {
      right_index = 0;
      right_points = next_right_points;
      right_ready = false;
    }
    
    double progress = static_cast<double>(left_index_) / (TRAJECTORY_SIZE - 1);

    // before the trajectory ends, ask for a new one
    bool right_needed = (progress > RIGHT_FOOT_REQUEST_PROGRESS); //right trajectory ends at 0.5 progress
    bool left_needed = (progress > LEFT_FOOT_REQUEST_PROGRESS); //left trajectory ends at 1.0 progress

    if (left_needed && !left_requested) {
      uint8_t foot_id = 0; // 0 = left, 1 = right
      requestTrajectory(foot_id);
      left_requested = true;
    }

    if (right_needed && !right_requested) {
      uint8_t foot_id = 1; // 0 = left, 1 = right
      requestTrajectory(foot_id);
      right_requested = true;
    }

    
    // here choose current point of the trajectory
    bipedal_robot::msg::FeetPositions feet_point_msg;
    feet_point_msg.left = left_points[left_index];
    feet_point_msg.right = right_points[right_index];
    feet_pub->publish(feet_point_msg);
    
    left_index++;
    
    if (progress > 0.5) start = true;
    if(start) right_index++;
  }

  void requestTrajectory(uint8_t foot_id)
  {
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Service not available");
      return;
    }

    auto request = std::make_shared<bipedal_robot::srv::FootTrajectory::Request>();
    request->foot_id = foot_id;

    auto future = client->async_send_request(request,
      [this, foot_id](rclcpp::Client<bipedal_robot::srv::FootTrajectory>::SharedFuture response) {
        auto res = response.get();
        if (foot_id == 0) {
          next_left_points = res->points;
          left_ready = true;
          left_requested = false;
        } else {
          next_right_points = res->points;
          right_ready = true;
          right_requested = false;
        }

        if (initialization && left_ready && right_ready) {
          left_points = next_left_points;
          right_points = next_right_points;
          left_ready = false;
          right_ready = false;
          initialization = false;
        }
      });
  }

  // Class members
  rclcpp::Publisher<bipedal_robot::msg::FeetPositions>::SharedPtr feet_pub;
  rclcpp::Client<bipedal_robot::srv::FootTrajectory>::SharedPtr client;
  rclcpp::TimerBase::SharedPtr timer;

  std::vector<geometry_msgs::msg::Point> left_points;  
  std::vector<geometry_msgs::msg::Point> right_points;
  std::vector<geometry_msgs::msg::Point> next_left_points;
  std::vector<geometry_msgs::msg::Point> next_right_points;


  double progress = 0.0;
  int left_index = 0;
  int right_index = 0;  
  bool left_ready = false;
  bool right_ready = false;
  bool left_requested = false;
  bool right_requested = false;
  bool initialization = true;
  bool start = false;
    
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryExecutorNode>());
  rclcpp::shutdown();
  return 0;
}

