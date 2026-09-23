#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

// ROS 2 C++ client library
#include <rclcpp/rclcpp.hpp>

// ROS 2 Message definitions (generated C++ structs/classes)
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

// Eigen is a C++ template library for linear algebra (matrices and vectors)
#include <Eigen/Dense>

// Using namespace inside an implementation file is fine and lets us write 20ms instead of std::chrono::milliseconds(20)
using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kNumericalTolerance = 1.0e-9;
}  // namespace

class PanTiltSubscriber : public rclcpp::Node
{
public:
  // ---------------------------------------------------------------------------
  // CONSTRUCTOR: Initializes base class and member variables.
  // The colon ':' begins the Member Initializer List (runs BEFORE constructor body).
  // ---------------------------------------------------------------------------
  PanTiltSubscriber()
  : Node("pan_tilt_node"),
    target_received_(false),
    transform_received_(false),
    a3_(1.0),
    projectile_velocity_(10.0),
    gravity_(9.81)
  {
    // Declare node parameters with default values
    this->declare_parameter<std::string>("target_topic", "/target_tf_position");
    this->declare_parameter<std::string>("output_topic", "/servo2_transform_matrix");
    this->declare_parameter<std::string>("loop_topic", "pan_tilt_command");
    this->declare_parameter<double>("a3", 1.0);
    this->declare_parameter<double>("p_vel", 10.0);
    this->declare_parameter<double>("g", 9.81);

    // Retrieve parameter values into member variables
    target_topic_ = this->get_parameter("target_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    loop_topic_ = this->get_parameter("loop_topic").as_string();

    a3_ = this->get_parameter("a3").as_double();
    projectile_velocity_ = this->get_parameter("p_vel").as_double();
    gravity_ = this->get_parameter("g").as_double();

    if (a3_ < 0.0 || projectile_velocity_ <= 0.0 || gravity_ <= 0.0) {
      throw std::invalid_argument(
              "Parameters must satisfy a3 >= 0, p_vel > 0, and g > 0");
    }

    // -------------------------------------------------------------------------
    // SUBSCRIBERS:
    // create_subscription<T> is a template method.
    // We pass a Lambda function [this](...) instead of std::bind for cleaner code.
    // [this] captures the class instance pointer so we can access class methods.
    // -------------------------------------------------------------------------
    target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      target_topic_, 10,
      [this](const geometry_msgs::msg::Point::SharedPtr msg) {
        this->target_callback(msg);
      });

    transform_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      output_topic_, 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        this->output_topic_callback(msg);
      });

    // -------------------------------------------------------------------------
    // PUBLISHER:
    // create_publisher returns a std::shared_ptr to a Publisher object.
    // -------------------------------------------------------------------------
    command_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
      loop_topic_, 10);

    // -------------------------------------------------------------------------
    // TIMER:
    // 20ms = 50 Hz. Runs the publish_commands callback periodically.
    // -------------------------------------------------------------------------
    timer_ = this->create_wall_timer(
      20ms,
      [this]() {this->publish_commands();});
  }

private:
  // ---------------------------------------------------------------------------
  // CALLBACK: Target Point
  // Takes a const SharedPtr reference: avoids copying the message data.
  // ---------------------------------------------------------------------------
  void target_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    // Eigen comma-initializer syntax to populate a 3D vector (x, y, z)
    target_xyz_ << msg->x, msg->y, msg->z;
    target_received_ = true;
  }

  // ---------------------------------------------------------------------------
  // CALLBACK: Gun Transform Matrix (4x4 flattened array)
  // ---------------------------------------------------------------------------
  void output_topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // Guard against malformed messages (must have 16 numbers for a 4x4 matrix)
    if (msg->data.size() != 16) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Expected 16 matrix values, received %zu", msg->data.size());
      return;
    }

    if (!std::all_of(
        msg->data.begin(), msg->data.end(), [](double value) {
          return std::isfinite(value);
        }))
    {
      RCLCPP_ERROR(this->get_logger(), "Transform matrix contains a non-finite value");
      return;
    }

    // ROS MultiArray is row-major. Fill the Eigen 4x4 matrix:
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        gun_transform_(row, col) = msg->data[row * 4 + col];   // can get x y and z from (0,3) (1,3) and (2,3)
      }
    }

    transform_received_ = true;
  }

  // ---------------------------------------------------------------------------
  // TIMER CALLBACK: Computes inverse kinematics / angles and publishes
  // ---------------------------------------------------------------------------
  void publish_commands()
  {
    // Guard clause: do nothing until both topics have provided data at least once
    if (!target_received_ || !transform_received_) {
      return;
    }

    // Work from the servo2/projectile origin rather than the base origin.
    const double x = target_xyz_(0) - gun_transform_(0, 3);
    const double y = target_xyz_(1) - gun_transform_(1, 3);
    const double z = target_xyz_(2) - gun_transform_(2, 3);

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Target position contains a non-finite value");
      return;
    }

    const double pan = std::atan2(y, x);
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
        this->get_logger(), *this->get_clock(), 2000,
        "Target is unreachable for the configured projectile velocity");
      return;
    }

    const double alpha = std::sqrt(std::max(0.0, discriminant));
    const double time_squared = 2*(velocity_height_term - alpha) / gravity_squared;

    if (time_squared < -kNumericalTolerance) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Projectile equation has no real flight-time solution");
      return;
    }

    const double flight_time = std::sqrt(std::max(0.0, time_squared));

    const double q = z + 0.5  * gravity_ * flight_time *flight_time;


    const double denominator =
      a3_ * horizontal_range -  projectile_velocity_ *flight_time * q;
    const double numerator =
      projectile_velocity_ * flight_time * horizontal_range + a3_ * q;
    const double tilt = std::atan2(numerator, denominator);


    const double tilt_deg = tilt * 180.0 / kPi;
    const double hardare_tilt_deg = tilt_deg - 90;
    const double pan_deg = pan * 180.0 / kPi;

    if (!std::isfinite(pan_deg) || !std::isfinite(tilt_deg)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Projectile calculation produced a non-finite command");
      return;
    }

    // 6. Build and publish message
    auto msg = std_msgs::msg::Int32MultiArray();

    // static_cast<int32_t> explicitly converts double to 32-bit signed integer
    msg.data = {
      static_cast<int32_t>(std::round(pan_deg)),
      static_cast<int32_t>(std::round(hardare_tilt_deg))
    };

    command_pub_->publish(msg);
  }

  // ---------------------------------------------------------------------------
  // MEMBER VARIABLES
  // Suffix '_' is the standard ROS C++ convention denoting class member variables.
  // ---------------------------------------------------------------------------
  Eigen::Vector3d target_xyz_;
  Eigen::Matrix4d gun_transform_;

  bool target_received_;
  bool transform_received_;

  double a3_;
  double projectile_velocity_;
  double gravity_;

  std::string target_topic_;
  std::string output_topic_;
  std::string loop_topic_;

  // Smart pointers to ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr transform_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// -----------------------------------------------------------------------------
// MAIN ENTRY POINT
// -----------------------------------------------------------------------------
int main(int argc, char * argv[])
{
  // 1. Initialize ROS 2 runtime
  rclcpp::init(argc, argv);

  // 2. Allocate the node on the heap managed by a shared pointer
  auto node = std::make_shared<PanTiltSubscriber>();

  // 3. Spin: Block this thread and process callbacks (timer + subscriptions)
  rclcpp::spin(node);

  // 4. Clean shutdown when Ctrl+C is pressed
  rclcpp::shutdown();
  return 0;
}
