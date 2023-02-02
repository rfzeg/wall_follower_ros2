/*
Rule based wall follower (ROS2)
Author: Roberto Zegers
Date: August 2022
*/

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>

#include <algorithm>
#include <iterator>

using std::placeholders::_1;
using namespace std::chrono_literals;

class ObstacleAvoidance : public rclcpp::Node {
public:
  ObstacleAvoidance() : Node("wall_follower_ros2") {

    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", default_qos,
        std::bind(&ObstacleAvoidance::laser_callback, this, _1));
    vel_msg_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // declare parameters and set default values
    this->declare_parameter("linear_x_velocity", 0.3);
    this->declare_parameter("angular_z_velocity", 0.2);
    this->declare_parameter("safety_distance", 0.5);
    this->declare_parameter("reverse_ranges_array", true);

    // get parameters values
    this->get_parameter("linear_x_velocity", linear_x_velocity_);
    this->get_parameter("angular_z_velocity", angular_z_velocity_);
    this->get_parameter("safety_distance", d);
    this->get_parameter("reverse_ranges_array", reverse_ranges_array);

    RCLCPP_INFO(this->get_logger(), "Wall follower node running");

    timer_ = this->create_wall_timer(
        100ms, std::bind(&ObstacleAvoidance::timerCallback, this));
  }

  ~ObstacleAvoidance() {}

  void sigint() {
    // stop the robot on shutdown
    geometry_msgs::msg::Twist zero_vel_msg;
    vel_msg_publisher_->publish(zero_vel_msg);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  double linear_x_velocity_;
  double angular_z_velocity_;
  // reverse ranges array if it is populated from left, positive angles, to
  // right, negative angles, wrt laser sensor.
  bool reverse_ranges_array;
  // if distance (m) reading is below the 'd' parameter value
  // a region is considered as blocked by an obstacle
  double d;
  // current minimum distance value on each zone (assuming 5 zones)
  float z[5];
  // array to keep track of the index that corresponds to the min. distance
  // value on each zone
  int indices[5];
  // how the robot should move based on the obstacles around it
  int drive_logic_state;
  // boolean flag to prevent driving without laser scanner data received
  bool readings_received = false;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_msg_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  geometry_msgs::msg::Twist vel_msg;

  void timerCallback() {
    if (readings_received == false) {
      // return early and skip the rest of the function
      return;
    } else { // define logic used to drive the robot
      // determine the movement state to drive the robot
      set_drive_logic_state();
      // fill in velocity message
      determine_vel_msg();
      // publish velocity message to the robot
      vel_msg_publisher_->publish(vel_msg);
    }
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr _scan_msg) {
    readings_received = true;
    // the total number of laser rays the laser range finder has
    size_t range_size = _scan_msg->ranges.size();
    // number of laser rays per laser range zone (assuming 5 zones)
    // size_t zone_size = static_cast<int>(range_size / 5);

    RCLCPP_INFO_ONCE(this->get_logger(), "Number of laser rays: [%zu]",
                     range_size);

    // initialize distance values on each zone to max laser scanner range
    z[0] = _scan_msg->range_max;
    z[1] = _scan_msg->range_max;
    z[2] = _scan_msg->range_max;
    z[3] = _scan_msg->range_max;
    z[4] = _scan_msg->range_max;

    // cycle trough all laser range rays
    // to determine the state of the environment surroundings
    for (auto i = 0u; i < _scan_msg->ranges.size(); ++i) {
      // laser rays to the far right side
      if (i < static_cast<unsigned>(_scan_msg->ranges.size() / 5)) {
        // get the distance reading to the closest object in the given zone
        if (_scan_msg->ranges[i] < z[0]) {
          z[0] = _scan_msg->ranges[i];
          indices[0] = i;
        }
      }
      // laser rays to the front-right side
      else if (i >= static_cast<unsigned>(_scan_msg->ranges.size() / 5) &&
               i < static_cast<unsigned>(_scan_msg->ranges.size() * 2 / 5)) {
        // get the distance reading to the closest object in the given zone
        if (_scan_msg->ranges[i] < z[1]) {
          z[1] = _scan_msg->ranges[i];
          indices[1] = i;
        }
      }
      // laser rays to the front
      else if (i >= static_cast<unsigned>(_scan_msg->ranges.size() * 2 / 5) &&
               i < static_cast<unsigned>(_scan_msg->ranges.size() * 3 / 5)) {
        // get the distance reading to the closest object in the given zone
        if (_scan_msg->ranges[i] < z[2]) {
          z[2] = _scan_msg->ranges[i];
          indices[2] = i;
        }
      }
      // laser rays to the front-left side
      else if (i >= static_cast<unsigned>(_scan_msg->ranges.size() * 3 / 5) &&
               i < static_cast<unsigned>(_scan_msg->ranges.size() * 4 / 5)) {
        // get the distance reading to the closest object in the given zone
        if (_scan_msg->ranges[i] < z[3]) {
          z[3] = _scan_msg->ranges[i];
          indices[3] = i;
        }
      }
      // laser rays to the far left side
      else if (i >= static_cast<unsigned>(_scan_msg->ranges.size() * 4 / 5) &&
               i <= static_cast<unsigned>(_scan_msg->ranges.size())) {
        // get the distance reading to the closest object in the given zone
        if (_scan_msg->ranges[i] < z[4]) {
          z[4] = _scan_msg->ranges[i];
          indices[4] = i;
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Ray index not found in range size");
      }
    } // end of for

    // this is just for the case the ranges array starts with angles at the left
    // side wrt to laser scanner
    if (reverse_ranges_array) {
      std::reverse(std::begin(z), std::end(z));
    }

    RCLCPP_DEBUG(this->get_logger(),
                 "Closest object to the far right: [%f], index: [%d] ", z[0],
                 indices[0]);
    RCLCPP_DEBUG(this->get_logger(),
                 "Closest object to the front-right: [%f], index: [%d]  ", z[1],
                 indices[1]);
    RCLCPP_DEBUG(this->get_logger(),
                 "Closest object to the front: [%f], index: [%d]  ", z[2],
                 indices[2]);
    RCLCPP_DEBUG(this->get_logger(),
                 "Closest object to the front-left: [%f], index: [%d]  ", z[3],
                 indices[3]);
    RCLCPP_DEBUG(this->get_logger(),
                 "Closest object to the far left: [%f], index: [%d]  ", z[4],
                 indices[4]);
  }

  /*
  Logic used to drive the robot (using 5 zones)
  it depends on the state of the environment surroundings
  To-do: evaluate zones as 'with obstacle' or 'without obstacle' only once
  */
  void set_drive_logic_state() {
    // logic block 1:
    if (z[0] > d && z[1] > d && z[2] > d && z[3] > d && z[4] > d) {
      RCLCPP_DEBUG(this->get_logger(), "Case 1: no obstacles detected");
      drive_logic_state = 0; // find obstacle: turn CW and move ahead
    } else if (z[0] > d && z[1] > d && z[2] < d && z[3] > d && z[4] > d) {
      RCLCPP_DEBUG(this->get_logger(), "Case 2: obstacle only in front zone");
      drive_logic_state = 1; // turn left
    } else if (z[0] > d && z[1] < d && z[2] > d && z[3] > d && z[4] > d) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Case 3: obstacle only in front-right zone");
      drive_logic_state = 1; // turn left
    } else if (z[0] > d && z[1] > d && z[2] > d && z[3] < d && z[4] > d) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Case 4: obstacle only in front-left zone");
      drive_logic_state = 0; // find obstacle: turn CW and move ahead
    } else if (z[0] > d && z[1] < d && z[2] < d && z[3] > d && z[4] > d) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Case 5: obstacle in front-right and front zone");
      drive_logic_state = 1; // turn left
    } else if (z[0] > d && z[1] > d && z[2] < d && z[3] < d && z[4] > d) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Case 6: obstacle in front and front-left zone");
      drive_logic_state = 1; // turn left
    } else if (z[0] > d && z[1] < d && z[2] < d && z[3] < d && z[4] > d) {
      RCLCPP_DEBUG(
          this->get_logger(),
          "Case 7: obstacle in front-right, front and front-left zone");
      drive_logic_state = 1; // turn left
    } else if (z[0] > d && z[1] < d && z[2] > d && z[3] < d && z[4] > d) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Case 8: obstacle in front-right and front-left zone");
      drive_logic_state = 3; // move slow straight ahead
    }
    // logic block 2:
    else if (z[0] < d && z[1] > d && z[2] > d && z[3] > d && z[4] > d) {
      RCLCPP_DEBUG(this->get_logger(), "Case 9: obstacle only in right zone");
      drive_logic_state =
          2; // keep moving straight ahead, obstacle only in right zone
    } else if (z[0] < d && z[1] > d && z[2] < d && z[3] > d && z[4] > d) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Case 10:  obstacle in right and front zone");
      drive_logic_state = 1; // turn left
    } else if (z[0] < d && z[1] < d && z[2] > d && z[3] > d && z[4] > d) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Case 11: obstacle in right and front-right zone");
      drive_logic_state = 1; // turn left
    } else if (z[0] < d && z[1] > d && z[2] > d && z[3] < d && z[4] > d) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Case 12: obstacle in right and front-left zone");
      drive_logic_state = 3; // move slow straight ahead
    } else if (z[0] < d && z[1] < d && z[2] < d && z[3] > d && z[4] > d) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Case 13: obstacle in right, front-right and front zone");
      drive_logic_state = 1; // turn left
    } else if (z[0] < d && z[1] > d && z[2] < d && z[3] < d && z[4] > d) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Case 14: obstacle in right, front and front-left zone");
      drive_logic_state = 1; // turn left
    } else if (z[0] < d && z[1] < d && z[2] < d && z[3] < d && z[4] > d) {
      RCLCPP_DEBUG(
          this->get_logger(),
          "Case 15: obst. in right, front-right, front and front-left zone");
      drive_logic_state = 1; // turn left
    } else if (z[0] < d && z[1] < d && z[2] > d && z[3] < d && z[4] > d) {
      RCLCPP_DEBUG(
          this->get_logger(),
          "Case 16: obstacle in right, front-right and front-left zone");
      drive_logic_state = 3; // move slow straight ahead
    }
    // logic block 3:
    else if (z[0] > d && z[1] > d && z[2] > d && z[3] > d && z[4] < d) {
      RCLCPP_DEBUG(this->get_logger(), "Case 17: obstacle only in left zone");
      drive_logic_state = 0; // find obstacle: turn CW and move ahead
    } else if (z[0] > d && z[1] > d && z[2] < d && z[3] > d && z[4] < d) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Case 18: obstacle in front and left zone");
      drive_logic_state = 0; // find obstacle: turn CW and move ahead
    } else if (z[0] > d && z[1] < d && z[2] > d && z[3] > d && z[4] < d) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Case 19: obstacle in front-right and left zone");
      drive_logic_state = 3; // move slow straight ahead
    } else if (z[0] > d && z[1] > d && z[2] > d && z[3] < d && z[4] < d) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Case 20: obstacle in front-left and left zone");
      drive_logic_state = 0; // find obstacle: turn CW and move ahead
    } else if (z[0] > d && z[1] < d && z[2] < d && z[3] > d && z[4] < d) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Case 21: obstacle in front-right, front and left zone");
      drive_logic_state = 0; // find obstacle: turn CW and move ahead
    } else if (z[0] > d && z[1] > d && z[2] < d && z[3] < d && z[4] < d) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Case 22: obstacle in front, front-left and left zone");
      drive_logic_state = 1; // turn left
    } else if (z[0] > d && z[1] < d && z[2] < d && z[3] < d && z[4] < d) {
      RCLCPP_DEBUG(
          this->get_logger(),
          "Case 23: obst. in front-right, front, front-left and left zone");
      drive_logic_state = 1; // turn left
    } else if (z[0] > d && z[1] < d && z[2] > d && z[3] < d && z[4] < d) {
      RCLCPP_DEBUG(
          this->get_logger(),
          "Case 24: obstacle in front-right, front-left and left zone");
      drive_logic_state = 3; // move slow straight ahead
    }
    // logic block 4:
    else if (z[0] < d && z[1] > d && z[2] > d && z[3] > d && z[4] < d) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Case 25: obstacle in right and left zone");
      drive_logic_state = 3; // move slow straight ahead
    } else if (z[0] < d && z[1] > d && z[2] < d && z[3] > d && z[4] < d) {
      RCLCPP_DEBUG(this->get_logger(), "Case 26: obstacle in right, front a no "
                                       "obstacles detectednd left zone");
      drive_logic_state = 1; // turn left
    } else if (z[0] < d && z[1] < d && z[2] > d && z[3] > d && z[4] < d) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Case 27: obstacle in right, front-right and left zone");
      drive_logic_state = 1; // turn left
    } else if (z[0] < d && z[1] > d && z[2] > d && z[3] < d && z[4] < d) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Case 28: obstacle in right, front-left and left zone");
      drive_logic_state = 0; // find obstacle: turn CW and move ahead
    } else if (z[0] < d && z[1] < d && z[2] < d && z[3] > d && z[4] < d) {
      RCLCPP_DEBUG(
          this->get_logger(),
          "Case 29: obstacle in right, front-right, front and left zone");
      drive_logic_state = 1; // turn left
    } else if (z[0] < d && z[1] > d && z[2] < d && z[3] < d && z[4] < d) {
      RCLCPP_DEBUG(
          this->get_logger(),
          "Case 30: obstacle in right, front, front-left, and left zone");
      drive_logic_state = 0; // find obstacle: turn CW and move ahead
    } else if (z[0] < d && z[1] < d && z[2] < d && z[3] < d && z[4] < d) {
      RCLCPP_DEBUG(this->get_logger(), "Case 31: obst. in right, front-right, "
                                       "front, front-left and left zone");
      drive_logic_state = 4; // reverse turning left
    } else if (z[0] < d && z[1] < d && z[2] > d && z[3] < d && z[4] < d) {
      RCLCPP_DEBUG(
          this->get_logger(),
          "Case 32: obst. in right, front-right, front-left and left zone");
      drive_logic_state = 3; // move slow straight ahead
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Unknown case");
    }
    RCLCPP_DEBUG(this->get_logger(), "Set drive logic state to: [%d]",
                 drive_logic_state);
  }

  /*
  Define velocity command based on state and fill in a Twist message
  */
  void determine_vel_msg() {
    RCLCPP_DEBUG(this->get_logger(), "Wall follower drive_logic_state: [%d]",
                 drive_logic_state);
    switch (drive_logic_state) {
    case 0:
      // move straight ahead: No obstacle detected
      vel_msg.linear.x = linear_x_velocity_;
      vel_msg.angular.z = 0.0;
      RCLCPP_DEBUG(this->get_logger(),
                   "Behaviour 0: Moving straight ahead, no obstacle detected.");
      break;

    case 1:
      // Turn left: obstacle in front-right and/or front and/or front-left
      vel_msg.linear.x = 0.0;
      vel_msg.angular.z = angular_z_velocity_;
      RCLCPP_DEBUG(this->get_logger(),
                   "Behaviour 1: Turn left, obstacle in front-right and/or "
                   "front and/or front-left zone.");

      break;

    case 2:
      // Keep moving straight ahead: obstacle only in right zone
      vel_msg.linear.x = linear_x_velocity_;
      vel_msg.angular.z = 0.0;
      RCLCPP_DEBUG(this->get_logger(), "Behaviour 2: Keep moving straight "
                                       "ahead, obstacle only in right zone");
      break;

    case 3:
      // move slow straight ahead: obstacle in front-right or right and
      // front-left or left zone but not in front
      vel_msg.linear.x = linear_x_velocity_;
      vel_msg.angular.z = 0.0;
      RCLCPP_DEBUG(
          this->get_logger(),
          "Behaviour 3: Keep moving straight ahead, obstacle in front-right or "
          "right and front-left or left zone but not in front");
      break;

    case 4:
      // Reverse turning left: obst. in right, front-right, front, front-left
      // and left zone
      vel_msg.linear.x = -linear_x_velocity_;
      vel_msg.angular.z = angular_z_velocity_;
      RCLCPP_DEBUG(this->get_logger(),
                   "Behaviour 4: Reverse turning left, obst. in right, "
                   "front-right, front, front-left and left zone");
      break;
    }
  }
};

namespace {
std::function<void(int)> sigint_handler;
void signal_handler(int signal) { sigint_handler(signal); }
} // namespace

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto obst_avoidance = std::make_shared<ObstacleAvoidance>();
  std::signal(SIGINT, signal_handler);
  sigint_handler = [obst_avoidance](int signal) {
    std::cout << "received signal " << signal << std::endl;
    obst_avoidance->sigint();
    rclcpp::shutdown();
    exit(signal);
  };
  rclcpp::spin(obst_avoidance);
  rclcpp::shutdown();
  return 0;
}