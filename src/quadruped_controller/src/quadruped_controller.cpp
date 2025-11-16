#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
// #include <control_msgs/msg/joint_trajectory_feedback.hpp>

class QuadrupedController : public rclcpp::Node {
public:
  QuadrupedController() : Node("quadruped_controller_node") {
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/quadruped_controller/joint_trajectory", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(2000), std::bind(&QuadrupedController::publish_trajectory, this));
  }

private:
  void publish_trajectory() {
    auto message = trajectory_msgs::msg::JointTrajectory();
    message.joint_names = {
      "front_left_hip", "front_left_knee", "front_left_ankle",
      "front_right_hip", "front_right_knee", "front_right_ankle",
      "rear_left_hip", "rear_left_knee", "rear_left_ankle",
      "rear_right_hip", "rear_right_knee", "rear_right_ankle"
    };

    // Exemple trajectoire simple : Lever front_left_knee à 0.5 rad, autres à 0
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point.time_from_start.sec = 1; // 1 seconde pour atteindre
    message.points.push_back(point);

    // Retour à zéro
    trajectory_msgs::msg::JointTrajectoryPoint point2;
    point2.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point2.time_from_start.sec = 2;
    message.points.push_back(point2);

    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Trajectoire publiée !");
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuadrupedController>());
  rclcpp::shutdown();
  return 0;
}