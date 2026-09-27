#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/transform_broadcaster.h"

class LineFitting : public rclcpp::Node
{
public:
  LineFitting()
  : Node("least_square_pred")
  {
    declare_parameter<std::string>("target_topic", "/target_tf_position");
    declare_parameter<std::string>("delta_t_topic", "/seperate_tof");
    declare_parameter<std::string>(
      "prediction_topic", "/predicted_target_position");
    declare_parameter<std::string>("parent_frame", "base_link");
    declare_parameter<std::string>(
      "predicted_frame", "target_prediction_tf");
    declare_parameter<int64_t>("window_size", 30);

    target_topic_ = get_parameter("target_topic").as_string();
    delta_t_topic_ = get_parameter("delta_t_topic").as_string();
    prediction_topic_ = get_parameter("prediction_topic").as_string();
    parent_frame_ = get_parameter("parent_frame").as_string();
    predicted_frame_ = get_parameter("predicted_frame").as_string();
    window_size_ = get_parameter("window_size").as_int();

    if (window_size_ < 3) {
      throw std::invalid_argument("window_size must be at least 3");
    }

    target_subscription_ =
      create_subscription<geometry_msgs::msg::PointStamped>(
      target_topic_, 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        target_callback(msg);
      });

    delta_t_subscription_ = create_subscription<std_msgs::msg::Float64>(
      delta_t_topic_, 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        delta_t_callback(msg);
      });

    prediction_publisher_ =
      create_publisher<geometry_msgs::msg::PointStamped>(prediction_topic_, 10);
    transform_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(
      get_logger(),
      "Waiting for target data on %s and flight time on %s",
      target_topic_.c_str(), delta_t_topic_.c_str());
  }

private:
  void delta_t_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data) || msg->data < 0.0) {
      RCLCPP_WARN(
        get_logger(), "Ignoring invalid flight time: %.6f s", msg->data);
      return;
    }

    delta_t_ = msg->data;

    // Re-predict immediately when a new flight time arrives and a fit exists.
    if (fit_valid_ && latest_stamp_ && !t_data_.empty()) {
      predict(t_data_.back() + *delta_t_);
    }
  }

  void target_callback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);

    if (!first_stamp_) {
      first_stamp_ = stamp;
    }

    latest_stamp_ = stamp;
    const double t = (stamp - *first_stamp_).seconds();

    if (!std::isfinite(t) ||
      !std::isfinite(msg->point.x) ||
      !std::isfinite(msg->point.y) ||
      !std::isfinite(msg->point.z))
    {
      RCLCPP_WARN(get_logger(), "Ignoring target with non-finite data");
      return;
    }

    add_measurement(t, msg->point.x, msg->point.y, msg->point.z);
  }

  void add_measurement(double t, double x, double y, double z)
  {
    t_data_.push_back(t);
    x_data_.push_back(x);
    y_data_.push_back(y);
    z_data_.push_back(z);

    if (t_data_.size() > static_cast<std::size_t>(window_size_)) {
      t_data_.pop_front();
      x_data_.pop_front();
      y_data_.pop_front();
      z_data_.pop_front();
    }

    if (t_data_.size() >= 3) {
      calculate_coefficients();
      if (delta_t_) {
        predict(t_data_.back() + *delta_t_);
      }
    }
  }

  void calculate_coefficients()
  {
    const int sample_count = static_cast<int>(t_data_.size());
    Eigen::MatrixXd observation(sample_count, 3);
    Eigen::VectorXd x(sample_count);
    Eigen::VectorXd y(sample_count);
    Eigen::VectorXd z(sample_count);

    for (int index = 0; index < sample_count; ++index) {
      const double t = t_data_[index];
      observation(index, 0) = 1.0;
      observation(index, 1) = t;
      observation(index, 2) = t * t;
      x(index) = x_data_[index];
      y(index) = y_data_[index];
      z(index) = z_data_[index];
    }

    auto solver = observation.colPivHouseholderQr();
    theta_x_ = solver.solve(x);
    theta_y_ = solver.solve(y);
    theta_z_ = solver.solve(z);
    fit_valid_ = theta_x_.allFinite() && theta_y_.allFinite() &&
      theta_z_.allFinite();

    if (!fit_valid_) {
      RCLCPP_WARN(get_logger(), "Least-squares fit produced invalid values");
    }
  }

  void predict(double prediction_time)
  {
    if (!fit_valid_ || !delta_t_ || !latest_stamp_) {
      return;
    }

    Eigen::Vector3d basis;
    basis << 1.0, prediction_time, prediction_time * prediction_time;

    Eigen::Vector3d predicted_position;
    predicted_position << basis.dot(theta_x_), basis.dot(theta_y_),
      basis.dot(theta_z_);

    if (!predicted_position.allFinite()) {
      RCLCPP_WARN(get_logger(), "Prediction produced invalid values");
      return;
    }

    const rclcpp::Time predicted_stamp =
      *latest_stamp_ + rclcpp::Duration::from_seconds(*delta_t_);
    publish_prediction(predicted_position, predicted_stamp);
  }

  void publish_prediction(
    const Eigen::Vector3d & position, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PointStamped point_msg;
    point_msg.header.stamp = stamp;
    point_msg.header.frame_id = parent_frame_;
    point_msg.point.x = position.x();
    point_msg.point.y = position.y();
    point_msg.point.z = position.z();
    prediction_publisher_->publish(point_msg);

    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header = point_msg.header;
    transform_msg.child_frame_id = predicted_frame_;
    transform_msg.transform.translation.x = position.x();
    transform_msg.transform.translation.y = position.y();
    transform_msg.transform.translation.z = position.z();
    transform_msg.transform.rotation.x = 0.0;
    transform_msg.transform.rotation.y = 0.0;
    transform_msg.transform.rotation.z = 0.0;
    transform_msg.transform.rotation.w = 1.0;
    transform_broadcaster_->sendTransform(transform_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
    target_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
    delta_t_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
    prediction_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  std::string target_topic_;
  std::string delta_t_topic_;
  std::string prediction_topic_;
  std::string parent_frame_;
  std::string predicted_frame_;
  int64_t window_size_{30};

  std::optional<double> delta_t_;
  std::optional<rclcpp::Time> first_stamp_;
  std::optional<rclcpp::Time> latest_stamp_;
  bool fit_valid_{false};

  Eigen::Vector3d theta_x_;
  Eigen::Vector3d theta_y_;
  Eigen::Vector3d theta_z_;
  std::deque<double> t_data_;
  std::deque<double> x_data_;
  std::deque<double> y_data_;
  std::deque<double> z_data_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineFitting>());
  rclcpp::shutdown();
  return 0;
}
