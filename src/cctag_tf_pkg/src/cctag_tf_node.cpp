#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <boost/ptr_container/ptr_list.hpp>

#include <cctag/CCTag.hpp>
#include <cctag/CCTagMarkersBank.hpp>
#include <cctag/Detection.hpp>
#include <cctag/Params.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>

namespace
{

double clamp_double(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

geometry_msgs::msg::Quaternion quaternion_from_axes(
  const tf2::Vector3 & x_axis,
  const tf2::Vector3 & y_axis,
  const tf2::Vector3 & z_axis)
{
  tf2::Matrix3x3 rotation(
    x_axis.x(), y_axis.x(), z_axis.x(),
    x_axis.y(), y_axis.y(), z_axis.y(),
    x_axis.z(), y_axis.z(), z_axis.z());

  tf2::Quaternion q;
  rotation.getRotation(q);
  q.normalize();

  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

}  // namespace

class CCTagTfNode : public rclcpp::Node
{
public:
  CCTagTfNode()
  : Node("cctag_tf_node"),
    params_(3),
    bank_(3)
  {
    image_topic_ = declare_parameter<std::string>(
      "image_topic", "/StereoNetNode/rectify_left_image");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/StereoNetNode/rectify_left_image/camera_info");
    output_pose_topic_ = declare_parameter<std::string>(
      "output_pose_topic", "/target_cc_position");
    output_point_topic_ = declare_parameter<std::string>(
      "output_point_topic", "/target_cc_point");
    output_id_topic_ = declare_parameter<std::string>(
      "output_id_topic", "/target_cc_id");
    parent_frame_ = declare_parameter<std::string>("parent_frame", "camera_optical_frame");
    target_frame_ = declare_parameter<std::string>("target_frame", "target_tf_cc");
    n_rings_ = declare_parameter<int>("n_rings", 3);
    target_id_ = declare_parameter<int>("target_id", -1);
    marker_outer_diameter_ = declare_parameter<double>("marker_outer_diameter", 0.055);
    use_cuda_ = declare_parameter<bool>("use_cuda", false);
    cctag_bank_file_ = declare_parameter<std::string>("cctag_bank_file", "");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    min_axis_pixels_ = declare_parameter<double>("min_axis_pixels", 4.0);
    max_axis_ratio_ = declare_parameter<double>("max_axis_ratio", 8.0);

    if (n_rings_ <= 0) {
      throw std::runtime_error("n_rings must be positive");
    }
    if (marker_outer_diameter_ <= 0.0) {
      throw std::runtime_error("marker_outer_diameter must be positive");
    }
    if (min_axis_pixels_ <= 0.0) {
      throw std::runtime_error("min_axis_pixels must be positive");
    }
    if (max_axis_ratio_ < 1.0) {
      throw std::runtime_error("max_axis_ratio must be >= 1.0");
    }

    params_ = cctag::Parameters(static_cast<std::size_t>(n_rings_));
    params_.setUseCuda(use_cuda_);

    if (cctag_bank_file_.empty()) {
      bank_ = cctag::CCTagMarkersBank(static_cast<std::size_t>(n_rings_));
    } else {
      bank_ = cctag::CCTagMarkersBank(cctag_bank_file_);
    }

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&CCTagTfNode::camera_info_callback, this, std::placeholders::_1));

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&CCTagTfNode::image_callback, this, std::placeholders::_1));

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(output_pose_topic_, 10);
    point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(output_point_topic_, 10);
    id_pub_ = create_publisher<std_msgs::msg::Int32>(output_id_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(
      get_logger(),
      "CCTag TF node started: image=%s camera_info=%s parent_frame=%s target_frame=%s rings=%d",
      image_topic_.c_str(),
      camera_info_topic_.c_str(),
      parent_frame_.c_str(),
      target_frame_.c_str(),
      n_rings_);
  }

private:
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (!latest_camera_info_) {
      RCLCPP_INFO(
        get_logger(),
        "Received camera_info on %s frame=%s fx=%.3f fy=%.3f",
        camera_info_topic_.c_str(),
        msg->header.frame_id.c_str(),
        msg->k[0],
        msg->k[4]);
    }
    latest_camera_info_ = msg;
  }

  cv::Mat image_to_gray(const sensor_msgs::msg::Image::SharedPtr & msg)
  {
    if (msg->encoding == "mono8") {
      return cv_bridge::toCvShare(msg, "mono8")->image.clone();
    }

    cv::Mat bgr;
    if (msg->encoding == "bgr8") {
      bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
    } else if (msg->encoding == "rgb8") {
      const auto rgb = cv_bridge::toCvShare(msg, "rgb8")->image;
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    } else if (msg->encoding == "nv12") {
      const auto nv12 = cv_bridge::toCvShare(msg, msg->encoding)->image;
      cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
    } else {
      bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
    }

    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    return gray;
  }

  const cctag::CCTag * select_marker(const cctag::CCTag::List & markers) const
  {
    const cctag::CCTag * best = nullptr;
    double best_area = -1.0;

    for (const auto & marker : markers) {
      if (marker.getStatus() != 1) {
        continue;
      }
      if (target_id_ >= 0 && marker.id() != target_id_) {
        continue;
      }

      const auto & ellipse = marker.rescaledOuterEllipse();
      const double a = std::abs(static_cast<double>(ellipse.a()));
      const double b = std::abs(static_cast<double>(ellipse.b()));
      const double major = std::max(a, b);
      const double minor = std::min(a, b);

      if (major < min_axis_pixels_ || minor < min_axis_pixels_) {
        continue;
      }
      if (major / minor > max_axis_ratio_) {
        continue;
      }

      const double area = major * minor;
      if (area > best_area) {
        best_area = area;
        best = &marker;
      }
    }

    return best;
  }

  geometry_msgs::msg::PoseStamped estimate_pose(
    const sensor_msgs::msg::Image::SharedPtr & image_msg,
    const sensor_msgs::msg::CameraInfo::SharedPtr & camera_info,
    const cctag::CCTag & marker) const
  {
    const auto & ellipse = marker.rescaledOuterEllipse();
    const double u = static_cast<double>(marker.x());
    const double v = static_cast<double>(marker.y());
    const double a = std::abs(static_cast<double>(ellipse.a()));
    const double b = std::abs(static_cast<double>(ellipse.b()));
    const double major = std::max(a, b);
    const double minor = std::min(a, b);

    const double fx = camera_info->k[0];
    const double fy = camera_info->k[4];
    const double cx = camera_info->k[2];
    const double cy = camera_info->k[5];
    const double focal = 0.5 * (fx + fy);
    const double outer_radius = 0.5 * marker_outer_diameter_;

    const double z = focal * outer_radius / major;
    const double x = (u - cx) * z / fx;
    const double y = (v - cy) * z / fy;

    const double ratio = clamp_double(minor / major, 0.0, 1.0);
    const double tilt = std::acos(ratio);
    const double angle = static_cast<double>(ellipse.angle());

    tf2::Vector3 x_axis(std::cos(angle), std::sin(angle), 0.0);
    x_axis.normalize();

    tf2::Vector3 normal(
      -std::sin(angle) * std::sin(tilt),
      std::cos(angle) * std::sin(tilt),
      std::cos(tilt));
    normal.normalize();

    tf2::Vector3 y_axis = normal.cross(x_axis);
    y_axis.normalize();
    x_axis = y_axis.cross(normal);
    x_axis.normalize();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = image_msg->header.stamp;
    pose.header.frame_id = parent_frame_.empty() ?
      (camera_info->header.frame_id.empty() ? image_msg->header.frame_id : camera_info->header.frame_id) :
      parent_frame_;

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation = quaternion_from_axes(x_axis, y_axis, normal);

    return pose;
  }

  void publish_outputs(const geometry_msgs::msg::PoseStamped & pose, int marker_id)
  {
    pose_pub_->publish(pose);

    geometry_msgs::msg::PointStamped point;
    point.header = pose.header;
    point.point = pose.pose.position;
    point_pub_->publish(point);

    std_msgs::msg::Int32 id_msg;
    id_msg.data = marker_id;
    id_pub_->publish(id_msg);

    if (!publish_tf_) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header = pose.header;
    transform.child_frame_id = target_frame_;
    transform.transform.translation.x = pose.pose.position.x;
    transform.transform.translation.y = pose.pose.position.y;
    transform.transform.translation.z = pose.pose.position.z;
    transform.transform.rotation = pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!latest_camera_info_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for camera_info on %s",
        camera_info_topic_.c_str());
      return;
    }

    cv::Mat gray;
    try {
      gray = image_to_gray(msg);
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Image conversion failed: %s", ex.what());
      return;
    } catch (const cv::Exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "OpenCV conversion failed: %s", ex.what());
      return;
    }

    cctag::CCTag::List markers;
    try {
      cctag::cctagDetection(
        markers,
        0,
        frame_id_++,
        gray,
        params_,
        bank_,
        false);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "CCTag detection failed: %s", ex.what());
      return;
    }

    const cctag::CCTag * marker = select_marker(markers);
    if (marker == nullptr) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "No reliable 3-ring CCTag detected");
      return;
    }

    const auto pose = estimate_pose(msg, latest_camera_info_, *marker);
    publish_outputs(pose, marker->id());

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "CCTag id=%d pose: x=%.3f y=%.3f z=%.3f frame=%s",
      marker->id(),
      pose.pose.position.x,
      pose.pose.position.y,
      pose.pose.position.z,
      pose.header.frame_id.c_str());
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string output_pose_topic_;
  std::string output_point_topic_;
  std::string output_id_topic_;
  std::string parent_frame_;
  std::string target_frame_;
  std::string cctag_bank_file_;
  int n_rings_;
  int target_id_;
  double marker_outer_diameter_;
  bool use_cuda_;
  bool publish_tf_;
  double min_axis_pixels_;
  double max_axis_ratio_;
  std::size_t frame_id_{0};

  cctag::Parameters params_;
  cctag::CCTagMarkersBank bank_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr id_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CCTagTfNode>());
  rclcpp::shutdown();
  return 0;
}
