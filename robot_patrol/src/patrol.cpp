#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class Patrol : public rclcpp::Node
{
public:
  Patrol()
  : Node("patrol")
  {
    laser_subscription_ =
      this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan",
        rclcpp::SensorDataQoS(),
        std::bind(
          &Patrol::laser_callback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Patrol laser reader started.");
  }

private:
  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kFrontHalfAngle = 20.0 * kPi / 180.0;

  void laser_callback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
  {
    // Start fresh for every scan.
    front_distance_ = std::numeric_limits<double>::infinity();
    best_distance_ = -std::numeric_limits<double>::infinity();
    best_angle_ = 0.0;

    for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
      const double distance = scan->ranges[i];

      // Ignore infinity, NaN, and readings outside the valid range.
      if (
        !std::isfinite(distance) ||
        distance < scan->range_min ||
        distance > scan->range_max)
      {
        continue;
      }

      // Calculate this ray's angle from the scan metadata.
      const double raw_angle =
        scan->angle_min +
        static_cast<double>(i) * scan->angle_increment;

      // Wrap the angle into the interval from -pi to +pi.
      const double angle =
        std::atan2(std::sin(raw_angle), std::cos(raw_angle));

      // Use only the front 180 degrees.
      if (std::abs(angle) > kPi / 2.0) {
        continue;
      }

      // Find the longest valid ray in the front 180 degrees.
      if (distance > best_distance_) {
        best_distance_ = distance;
        best_angle_ = angle;
      }

      // Find the shortest valid ray within 20 degrees of straight ahead.
      if (std::abs(angle) <= kFrontHalfAngle) {
        front_distance_ = std::min(front_distance_, distance);
      }
    }

    // Infinity here means the narrow front sector had no valid readings.
    if (!std::isfinite(front_distance_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "No valid readings in the narrow front sector.");
      return;
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Front minimum=%.2f m | Best ray=%.2f m at %.1f deg",
      front_distance_,
      best_distance_,
      best_angle_ * 180.0 / kPi);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
    laser_subscription_;

  double front_distance_ = std::numeric_limits<double>::infinity();
  double best_distance_ = -std::numeric_limits<double>::infinity();
  double best_angle_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}