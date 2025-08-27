#include <rclcpp/rclcpp.hpp>
#include <bipedal_robot/msg/feetpositions.hpp>
#include <bipedal_robot/msg/jointsangles.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

#define A 5.0
#define E 10.0
#define F 10.0

class IKNode : public rclcpp::Node {
public:
    IKNode() : Node("ik_node") {
        // Publisher
        joint_pub_ = this->create_publisher<Bipedal_Robot::msg::JointAngles>("joints_angles", 10);

        // Subscriber
        feet_sub_ = this->create_subscription<Bipedal_Robot::msg::FeetPositions>(
            "feet_position", 10,
            std::bind(&IKNode::feetCallback, this, std::placeholders::_1)
        );
    }

private:
    rclcpp::Subscription<Bipedal_Robot::msg::FeetPositions>::SharedPtr feet_sub_;
    rclcpp::Publisher<Bipedal_Robot::msg::JointAngles>::SharedPtr joint_pub_;

    struct Angles {
        double gamma;
        double theta;
        double phi;
        bool valid;
    };

    // =======================================
    void feetCallback(const Bipedal_Robot::msg::FeetPositions::SharedPtr msg) {
        Angles ang_L = inverseKinematics(msg->left.x, msg->left.y, msg->left.z);
        Angles ang_R = inverseKinematics(msg->right.x, msg->right.y, msg->right.z);

        if (!ang_L.valid || !ang_R.valid) {
            RCLCPP_WARN(this->get_logger(), "Posizione piede non raggiungibile");
            return;
        }

        Bipedal_Robot::msg::JointAngles joint_msg;
        joint_msg.header.stamp = this->get_clock()->now();
        joint_msg.name = {"hip_joint_L", "knee_joint_L", "ankle_joint_L", "hip_joint_R", "knee_joint_R", "ankle_joint_R"};
        joint_msg.position = {ang_L.gamma, ang_L.theta, ang_L.phi, ang_R.gamma, ang_R.theta, ang_R.phi};

        joint_pub_->publish(joint_msg);
    }

    inline double clamp(double x, double minVal=-1.0, double maxVal=1.0) {
        return std::max(minVal, std::min(maxVal, x));
    }

    Angles inverseKinematics(double X, double Y, double Z) {
        double C2 = Y*Y + Z*Z;
        if (C2 < A_*A_) return {0,0,0,false};

        double C = std::sqrt(C2);
        double D = std::sqrt(C*C - A_*A_);
        double delta = std::atan2(Y, Z);
        double epsilon = std::atan2(D, A_);
        double omega = delta + epsilon;
        double gamma = omega - M_PI/2.0;

        double Dz = D * std::cos(gamma);
        double Ez = E_ * std::cos(gamma);
        double Fz = F_ * std::cos(gamma);

        double G = std::sqrt(Dz*Dz + X*X);
        double cosPhi = clamp((G*G - Ez*Ez - Fz*Fz) / (-2.0*Ez*Fz));
        double phi = std::acos(cosPhi);

        double alpha = std::atan2(X, Dz);
        double sinArg = clamp((Fz * std::sin(phi)) / G);
        double beta = std::asin(sinArg);
        double theta = alpha + beta;

        return {gamma, theta, phi, true};
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IKNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




