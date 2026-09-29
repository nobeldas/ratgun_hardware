#include <cmath>
#include <cstdint>
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

class KalmanPrediction : public rclcpp::Node
{
public:
  KalmanPrediction()
  : Node("kalman_prediction")
  {
    declare_parameter<std::string>("target_topic", "/target_tf_position");
    declare_parameter<std::string>("delta_t_topic", "/seperate_tof");
    declare_parameter<std::string>(
      "prediction_topic", "/kalman_predicted_target_position");
    declare_parameter<std::string>("parent_frame", "base_link");
    declare_parameter<std::string>(
      "predicted_frame", "target_kalman_prediction_tf");
    declare_parameter<double>("initial_position_variance", 0.1);
    declare_parameter<double>("initial_velocity_variance", 1.0);
    declare_parameter<double>("process_acceleration_variance", 1.0);
    declare_parameter<double>("measurement_variance_x", 0.05);
    declare_parameter<double>("measurement_variance_y", 0.05);
    declare_parameter<double>("maximum_measurement_gap", 1.0);

    target_topic_ = get_parameter("target_topic").as_string();
    delta_t_topic_ = get_parameter("delta_t_topic").as_string();
    prediction_topic_ = get_parameter("prediction_topic").as_string();
    parent_frame_ = get_parameter("parent_frame").as_string();
    predicted_frame_ = get_parameter("predicted_frame").as_string();
    initial_position_variance_ =
      get_parameter("initial_position_variance").as_double();
    initial_velocity_variance_ =
      get_parameter("initial_velocity_variance").as_double();
    process_acceleration_variance_ =
      get_parameter("process_acceleration_variance").as_double();
    measurement_variance_x_ =
      get_parameter("measurement_variance_x").as_double();
    measurement_variance_y_ =
      get_parameter("measurement_variance_y").as_double();
    maximum_measurement_gap_ =
      get_parameter("maximum_measurement_gap").as_double();

    validate_parameters();

    measurement_matrix_.setZero();
    measurement_matrix_(0, 0) = 1.0;
    measurement_matrix_(1, 1) = 1.0;
    measurement_noise_.setZero();
    measurement_noise_(0, 0) = measurement_variance_x_;
    measurement_noise_(1, 1) = measurement_variance_y_;

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
  void validate_parameters() const
  {
    if (initial_position_variance_ <= 0.0 ||
      initial_velocity_variance_ <= 0.0 ||
      process_acceleration_variance_ < 0.0 ||
      measurement_variance_x_ <= 0.0 ||
      measurement_variance_y_ <= 0.0 ||
      maximum_measurement_gap_ <= 0.0)
    {
      throw std::invalid_argument(
              "Kalman variances must be non-negative, measurement and initial "
              "variances must be positive, and maximum_measurement_gap must "
              "be positive");
    }
  }

  void delta_t_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data) || msg->data < 0.0) {
      RCLCPP_WARN(
        get_logger(), "Ignoring invalid flight time: %.6f s", msg->data);
      return;
    }

    delta_t_ = msg->data;
    publish_prediction();
  }

  void target_callback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    const double measured_x = msg->point.x;
    const double measured_y = msg->point.y;
    const double measured_z = msg->point.z;

    if (!std::isfinite(measured_x) || !std::isfinite(measured_y) ||
      !std::isfinite(measured_z))
    {
      RCLCPP_WARN(get_logger(), "Ignoring target with non-finite coordinates");
      return;
    }

    const rclcpp::Time stamp(msg->header.stamp);
    if (!initialized_) {
      initialize_filter(measured_x, measured_y, measured_z, stamp);
      publish_prediction();
      return;
    }

    const double measurement_dt = (stamp - *latest_stamp_).seconds();
    if (!std::isfinite(measurement_dt) || measurement_dt <= 0.0) {
      RCLCPP_WARN(
        get_logger(), "Ignoring target with a non-increasing timestamp");
      return;
    }

    if (measurement_dt > maximum_measurement_gap_) {
      RCLCPP_WARN(
        get_logger(),
        "Measurement gap %.3f s exceeded %.3f s; resetting the filter",
        measurement_dt, maximum_measurement_gap_);
      initialize_filter(measured_x, measured_y, measured_z, stamp);
      publish_prediction();
      return;
    }

    predict_filter(measurement_dt);
    update_filter(measured_x, measured_y);
    latest_z_ = measured_z;
    latest_stamp_ = stamp;
    publish_prediction();
  }

  void initialize_filter(
    double x, double y, double z, const rclcpp::Time & stamp)
  {
    state_ << x, y, 0.0, 0.0;
    covariance_.setZero();
    covariance_(0, 0) = initial_position_variance_;
    covariance_(1, 1) = initial_position_variance_;
    covariance_(2, 2) = initial_velocity_variance_;
    covariance_(3, 3) = initial_velocity_variance_;
    latest_z_ = z;
    latest_stamp_ = stamp;
    initialized_ = true;
  }

  Eigen::Matrix4d transition_matrix(double dt) const
  {
    Eigen::Matrix4d transition = Eigen::Matrix4d::Identity();
    transition(0, 2) = dt;
    transition(1, 3) = dt;
    return transition;
  }

  Eigen::Matrix4d process_noise(double dt) const
  {
    const double dt_squared = dt * dt;
    const double dt_cubed = dt_squared * dt;
    const double dt_fourth = dt_squared * dt_squared;

    Eigen::Matrix4d noise = Eigen::Matrix4d::Zero();
    noise(0, 0) = dt_fourth / 4.0;
    noise(0, 2) = dt_cubed / 2.0;
    noise(1, 1) = dt_fourth / 4.0;
    noise(1, 3) = dt_cubed / 2.0;
    noise(2, 0) = dt_cubed / 2.0;
    noise(2, 2) = dt_squared;
    noise(3, 1) = dt_cubed / 2.0;
    noise(3, 3) = dt_squared;
    return noise * process_acceleration_variance_;
  }

  void predict_filter(double dt)
  {
    const Eigen::Matrix4d transition = transition_matrix(dt);
    state_ = transition * state_;
    covariance_ = transition * covariance_ * transition.transpose() +
      process_noise(dt);
  }

  void update_filter(double measured_x, double measured_y)
  {
    Eigen::Vector2d measurement;
    measurement << measured_x, measured_y;

    const Eigen::Vector2d innovation =
      measurement - measurement_matrix_ * state_;
    const Eigen::Matrix2d innovation_covariance =
      measurement_matrix_ * covariance_ * measurement_matrix_.transpose() +
      measurement_noise_;

    const Eigen::LDLT<Eigen::Matrix2d> decomposition(
      innovation_covariance);
    if (decomposition.info() != Eigen::Success) {
      RCLCPP_WARN(get_logger(), "Kalman innovation covariance is invalid");
      return;
    }

    const Eigen::Matrix<double, 4, 2> covariance_measurement =
      covariance_ * measurement_matrix_.transpose();
    const Eigen::Matrix<double, 4, 2> gain =
      covariance_measurement *
      decomposition.solve(Eigen::Matrix2d::Identity());

    state_ += gain * innovation;

    const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
    const Eigen::Matrix4d correction = identity - gain * measurement_matrix_;
    covariance_ = correction * covariance_ * correction.transpose() +
      gain * measurement_noise_ * gain.transpose();
  }

  void publish_prediction()
  {
    if (!initialized_ || !delta_t_ || !latest_stamp_) {
      return;
    }

    const Eigen::Vector4d predicted_state =
      transition_matrix(*delta_t_) * state_;
    if (!predicted_state.allFinite() || !std::isfinite(latest_z_)) {
      RCLCPP_WARN(get_logger(), "Kalman prediction produced invalid values");
      return;
    }

    const rclcpp::Time predicted_stamp =
      *latest_stamp_ + rclcpp::Duration::from_seconds(*delta_t_);

    geometry_msgs::msg::PointStamped point_msg;
    point_msg.header.stamp = predicted_stamp;
    point_msg.header.frame_id = parent_frame_;
    point_msg.point.x = predicted_state(0);
    point_msg.point.y = predicted_state(1);
    point_msg.point.z = latest_z_;
    prediction_publisher_->publish(point_msg);

    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header = point_msg.header;
    transform_msg.child_frame_id = predicted_frame_;
    transform_msg.transform.translation.x = point_msg.point.x;
    transform_msg.transform.translation.y = point_msg.point.y;
    transform_msg.transform.translation.z = point_msg.point.z;
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
  double initial_position_variance_{0.1};
  double initial_velocity_variance_{1.0};
  double process_acceleration_variance_{1.0};
  double measurement_variance_x_{0.05};
  double measurement_variance_y_{0.05};
  double maximum_measurement_gap_{1.0};

  bool initialized_{false};
  double latest_z_{0.0};
  std::optional<double> delta_t_;
  std::optional<rclcpp::Time> latest_stamp_;

  Eigen::Vector4d state_{Eigen::Vector4d::Zero()};
  Eigen::Matrix4d covariance_{Eigen::Matrix4d::Zero()};
  Eigen::Matrix<double, 2, 4> measurement_matrix_;
  Eigen::Matrix2d measurement_noise_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KalmanPrediction>());
  rclcpp::shutdown();
  return 0;
}
