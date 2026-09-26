#include <memory>
#include <optional>
#include <Eigen/Dense>
#include <vector>
#include <deque>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"


class LineFitting : public rclcpp::Node
{
public:
  LineFitting()
  : Node("least_square_pred") ,window_size(30)
  {
    target_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/target_tf_position",
      10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        target_callback(msg);
      });

    delta_t_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/delta_t",
      10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        delta_t_callback(msg);
      }); // i want to get delta t which will be used to get a future position from the s(tcurr + delta_t) so add a appropriate message type and a calback;

  }

private:

  void delta_t_callback(); 

  void target_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);
    if (!first_stamp_) {
      first_stamp_ = stamp;
    }
    const double t = (stamp - *first_stamp_).seconds();

    target_xyz << msg->point.x, msg->point.y, msg->point.z;
    addMeasurement(t, msg->point.x, msg->point.y, msg->point.z);
  }

  void addMeasurement(double t, double x, double y, double z)
  {
    t_data.push_back(t);
    x_data.push_back(x);
    y_data.push_back(y);
    z_data.push_back(z);

    if (t_data.size()> window_size){
      t_data.pop_front();
      x_data.pop_front();
      y_data.pop_front();
      z_data.pop_front();
    }

    if (t_data.size()>= 3){
      calculateTheta();
    }
  }
  // ============================================================
    // Calculate theta
    //
    // Book:
    //
    // theta_hat = (H^T H)^-1 H^T x
    // ============================================================

    void calculateTheta()
    {
        int N = t_data.size();  // window size max

        // Observation matrix
        //
        //       [ 1  t0   t0^2 ]
        //       [ 1  t1   t1^2 ]
        // H  =  [ .   .     .  ]
        //       [ .   .     .  ]
        //       [ 1  tN   tN^2 ]
        //

        Eigen::MatrixXd H(N, 3);
        Eigen::VectorXd X(N);
        Eigen::VectorXd Y(N);
        Eigen::VectorXd Z(N);

        for (int n = 0; n < N; n++)
        {
            double t = t_data[n];

            H(n, 0) = 1.0;
            H(n, 1) = t;
            H(n, 2) = t * t;

            X(n) = x_data[n];
            Y(n) = y_data[n];
            Z(n) = z_data[n];
        }

        // ========================================================
        // Same equation as the book:
        //
        // θ_hat = (HᵀH)^-1 Hᵀx
        // ========================================================

        Eigen::Matrix3d HTH_inverse =
            (H.transpose() * H).inverse();

        theta_x =
            HTH_inverse * H.transpose() * X;

        theta_y =
            HTH_inverse * H.transpose() * Y;

        theta_z =
            HTH_inverse * H.transpose() * Z;
            
        s(t + t_pred);
    }


    // ============================================================
    // Calculate fitted/predicted position s_hat(t)
    // ============================================================

    void s(double t)
    {
        Eigen::Vector3d h;

        h << 1.0,
             t,
             t * t;


    

        // x_hat(t) = [1 t t²] θ_x
        pred_xyz[0] = h.dot(theta_x);

        // y_hat(t) = [1 t t²] θ_y
        pred_xyz[1] = h.dot(theta_y);

        // z_hat(t) = [1 t t²] θ_z
        pred_xyz[2] = h.dot(theta_z);


    }


  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr delta_t_subscription_;
  std::optional<rclcpp::Time> first_stamp_;
  Eigen::Vector3d target_xyz;
  Eigen::Vector3d pred_xyz;
  Eigen::Vector3d theta_x;
  Eigen::Vector3d theta_y;
  Eigen::Vector3d theta_z;
  std::int64_t window_size;
  std::deque<double> t_pred;
  std::deque<double> t_data;
  std::deque<double> x_data;
  std::deque<double> y_data;
  std::deque<double> z_data;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineFitting>());
  rclcpp::shutdown();
  return 0;
}
