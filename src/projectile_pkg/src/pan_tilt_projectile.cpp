#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

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
    a1_(1.0),
    a2_(1.0),
    a3_(1.0)
  {
    // Declare node parameters with default values
    this->declare_parameter<std::string>("target_topic", "/target_tf_position");
    this->declare_parameter<std::string>("output_topic", "/gun_transform_matrix");
    this->declare_parameter<std::string>("loop_topic", "pan_tilt_command");
    this->declare_parameter<double>("a1", 1.0);
    this->declare_parameter<double>("a2", 1.0);
    this->declare_parameter<double>("a3", 1.0);

    // Retrieve parameter values into member variables
    target_topic_ = this->get_parameter("target_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    loop_topic_ = this->get_parameter("loop_topic").as_string();

    a1_ = this->get_parameter("a1").as_double();
    a2_ = this->get_parameter("a2").as_double();
    a3_ = this->get_parameter("a3").as_double();

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
      [this]() { this->publish_commands(); });
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

    // ROS MultiArray is row-major. Fill the Eigen 4x4 matrix:
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        gun_transform_(row, col) = msg->data[row * 4 + col];
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

    // Target coordinates extracted from our Eigen 3D vector
    const double x = target_xyz_(0);
    const double y = target_xyz_(1);
    const double z = target_xyz_(2);

    // 1. Compute pan angle (yaw around Z axis)
    const double pan = std::atan2(y, x);

    // 2. Compute horizontal distance projected onto the pan plane
    const double r = x * std::cos(pan) + y * std::sin(pan);

    // 3. Compute vertical displacement adjusted for arm offsets
    const double h = z - a1_ - a2_;

    // 4. Total Euclidean distance in the pan plane
    const double R = std::hypot(r, h); // Cleaner and numerically safer than std::sqrt(r*r + h*h)

    // Safety check: Avoid division by zero
    if (R == 0.0) {
      RCLCPP_WARN(this->get_logger(), "Target distance R is zero.");
      return;
    }

    // Safety check: acos domain [-1.0, 1.0]
    const double acos_argument = a3_ / R;
    if (acos_argument < -1.0 || acos_argument > 1.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "Target outside valid tilt geometry. a3/R = %.4f", acos_argument);
      return;
    }

    // 5. Tilt calculation
    const double alpha = std::atan2(r, h);
    const double tilt1 = std::acos(acos_argument) - alpha;

    // Convert radians to degrees (M_PI comes from <cmath>)
    const double pan_deg = pan * (180.0 / M_PI);
    const double tilt1_deg = tilt1 * (180.0 / M_PI);

    // 6. Build and publish message
    auto msg = std_msgs::msg::Int32MultiArray();

    // static_cast<int32_t> explicitly converts double to 32-bit signed integer
    msg.data = {
      static_cast<int32_t>(std::round(pan_deg)),
      static_cast<int32_t>(std::round(tilt1_deg))
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

  double a1_;
  double a2_;
  double a3_;

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