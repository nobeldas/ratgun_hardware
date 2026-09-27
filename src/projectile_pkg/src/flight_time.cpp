#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kNumericalTolerance = 1.0e-9;
}  // namespace

class FlightTime : public rclcpp::Node
{
public:
  FlightTime()
  : Node("flight_time"),
    target_received_(false),
    transform_received_(false),
    target_x_(0.0),
    target_y_(0.0),
    target_z_(0.0),
    servo_x_(0.0),
    servo_y_(0.0),
    servo_z_(0.0)
  {
    declare_parameter<std::string>("target_topic", "/target_tf_position");
    declare_parameter<std::string>("transform_topic", "/servo2_transform_matrix");
    declare_parameter<std::string>("tof_topic", "/seperate_tof");
    declare_parameter<double>("a3", 1.0);
    declare_parameter<double>("p_vel", 10.0);
    declare_parameter<double>("g", 9.81);

    const auto target_topic = get_parameter("target_topic").as_string();
    const auto transform_topic = get_parameter("transform_topic").as_string();
    const auto tof_topic = get_parameter("tof_topic").as_string();
    a3_ = get_parameter("a3").as_double();
    projectile_velocity_ = get_parameter("p_vel").as_double();
    gravity_ = get_parameter("g").as_double();

    if (a3_ < 0.0 || projectile_velocity_ <= 0.0 || gravity_ <= 0.0) {
      throw std::invalid_argument(
              "Parameters must satisfy a3 >= 0, p_vel > 0, and g > 0");
    }

    target_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
      target_topic,
      10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        target_x_ = msg->point.x;
        target_y_ = msg->point.y;
        target_z_ = msg->point.z;
        target_received_ = true;
      });

    transform_subscription_ =
      create_subscription<std_msgs::msg::Float64MultiArray>(
      transform_topic,
      10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        transform_callback(msg);
      });

    tof_publisher_ = create_publisher<std_msgs::msg::Float64>(tof_topic, 10);
    timer_ = create_wall_timer(20ms, [this]() {publish_flight_time();});
  }

private:
  void transform_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 16) {
      RCLCPP_ERROR(
        get_logger(), "Expected 16 matrix values, received %zu", msg->data.size());
      return;
    }

    if (!std::all_of(
        msg->data.begin(), msg->data.end(), [](double value) {
          return std::isfinite(value);
        }))
    {
      RCLCPP_ERROR(get_logger(), "Transform matrix contains a non-finite value");
      return;
    }

    servo_x_ = msg->data[3];
    servo_y_ = msg->data[7];
    servo_z_ = msg->data[11];
    transform_received_ = true;
  }

  void publish_flight_time()
  {
    if (!target_received_ || !transform_received_) {
      return;
    }

    const double x = target_x_ - servo_x_;
    const double y = target_y_ - servo_y_;
    const double z = target_z_ - servo_z_;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Target position contains a non-finite value");
      return;
    }

    const double horizontal_range = std::hypot(x, y);
    const double velocity_squared = projectile_velocity_ * projectile_velocity_;
    const double gravity_squared = gravity_ * gravity_;
    const double velocity_height_term = velocity_squared - z * gravity_;
    const double discriminant =
      velocity_height_term * velocity_height_term -
      gravity_squared *
      (horizontal_range * horizontal_range + z * z - a3_ * a3_);

    if (discriminant < -kNumericalTolerance) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Target is unreachable for the configured projectile velocity");
      return;
    }

    const double alpha = std::sqrt(std::max(0.0, discriminant));
    const double time_squared =
      2.0 * (velocity_height_term - alpha) / gravity_squared;

    if (time_squared < -kNumericalTolerance) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Projectile equation has no real flight-time solution");
      return;
    }

    const double flight_time = std::sqrt(std::max(0.0, time_squared));
    if (!std::isfinite(flight_time)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Projectile calculation produced a non-finite flight time");
      return;
    }

    std_msgs::msg::Float64 msg;
    msg.data = flight_time;
    tof_publisher_->publish(msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
    target_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    transform_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tof_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool target_received_;
  bool transform_received_;
  double target_x_;
  double target_y_;
  double target_z_;
  double servo_x_;
  double servo_y_;
  double servo_z_;
  double a3_;
  double projectile_velocity_;
  double gravity_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightTime>());
  rclcpp::shutdown();
  return 0;
}
