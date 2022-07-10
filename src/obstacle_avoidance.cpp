// Rule based obstacle avoidance

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

class ObstacleAvoidance : public rclcpp::Node {
public:
  ObstacleAvoidance() : Node("ObstacleAvoidance") {

    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "laser_scan", default_qos,
        std::bind(&ObstacleAvoidance::topic_callback, this, _1));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // declare parameters and set default values
    this->declare_parameter("linear_x_velocity", 0.3);
    this->declare_parameter("angular_z_velocity", 0.2);
    this->declare_parameter("safety_distance", 5.0);
    // get parameters values
    this->get_parameter("linear_x_velocity", linear_x_velocity_);
    this->get_parameter("angular_z_velocity", angular_z_velocity_);
    this->get_parameter("safety_distance", safety_distance_);

    RCLCPP_INFO(this->get_logger(), "Obstacle avoidance running");

    timer_ = this->create_wall_timer(
        1000ms, std::bind(&ObstacleAvoidance::respond, this));
  }
  void respond() {}

private:
  rclcpp::TimerBase::SharedPtr timer_;
  double linear_x_velocity_;
  double angular_z_velocity_;
  double safety_distance_;

  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
    // 200 readings, from right to left, from -57 to 57 degress
    float min = 10;
    for (int i = 0; i < 200; i++) {
      float current = _msg->ranges[i];
      if (current < min) {
        min = current;
      }
    }
    // calculate new velocity cmd
    auto message = this->calculateVelMsg(min);
    publisher_->publish(message);
  }
  geometry_msgs::msg::Twist calculateVelMsg(float distance) {
    auto msg = geometry_msgs::msg::Twist();
    // logic
    RCLCPP_DEBUG(this->get_logger(), "Distance reading is: '%f'", distance);
    // basic rule
    if (distance < safety_distance_) {
      // turn
      RCLCPP_DEBUG(this->get_logger(),
                   "Turning in place with an angular velocity of: '%f'",
                   angular_z_velocity_);
      msg.linear.x = 0;
      msg.angular.z = angular_z_velocity_;
    } else {
      // go straight ahead
      RCLCPP_DEBUG(this->get_logger(),
                   "Moving forward in a straight line with a velocity of: '%f'",
                   linear_x_velocity_);
      msg.linear.x = linear_x_velocity_;
      msg.angular.z = 0;
    }
    return msg;
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoidance>());
  rclcpp::shutdown();
  return 0;
}