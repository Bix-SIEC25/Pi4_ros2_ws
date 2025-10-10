#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/motors_feedback.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <cmath>
#include <memory>

using std::placeholders::_1;

class WheelOdometryNode : public rclcpp::Node
{
public:
  WheelOdometryNode()
  : Node("wheel_odom_node"),
    x_(0.0),
    y_(0.0),
    th_(0.0),
    first_msg_(true)
  {
    // --- Paramètres ---
    wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.05);       // m
    wheel_separation_ = this->declare_parameter<double>("wheel_separation", 0.30); // m
    ticks_per_rev_ = this->declare_parameter<int>("ticks_per_rev", 36);          // pulses / tour

    frame_id_ = this->declare_parameter<std::string>("frame_id", "odom");
    child_frame_id_ = this->declare_parameter<std::string>("child_frame_id", "base_link");

    // --- Publisher / Subscriber ---
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    motors_feedback_sub_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
      "motors_feedback", 10,
      std::bind(&WheelOdometryNode::motorsFeedbackCallback, this, _1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(),
                "wheel_odom_node READY (R=%.3f m, L=%.3f m, ticks/rev=%d)",
                wheel_radius_, wheel_separation_, ticks_per_rev_);
  }

private:
  void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & msg)
  {
    // Temps courant (pas de header dans MotorsFeedback, donc on prend now())
    rclcpp::Time current_time = this->now();

    if (first_msg_) {
      last_time_ = current_time;
      first_msg_ = false;
      return;
    }

    double dt = (current_time - last_time_).seconds();
    if (dt <= 0.0) {
      // bizarre mais on évite les divisions par 0
      last_time_ = current_time;
      return;
    }
    last_time_ = current_time;

    // --- 1) Distances des roues depuis le dernier message ---
    // Pulses -> tours -> distance (2πR)
    int left_ticks = static_cast<int>(msg.left_rear_odometry);
    int right_ticks = static_cast<int>(msg.right_rear_odometry);

    double meters_per_tick = (2.0 * M_PI * wheel_radius_) / static_cast<double>(ticks_per_rev_);

    double dist_left = static_cast<double>(left_ticks)  * meters_per_tick;
    double dist_right = static_cast<double>(right_ticks) * meters_per_tick;

    // --- 2) Cinématique diff-drive ---
    double d_center = (dist_left + dist_right) / 2.0;
    double d_theta  = (dist_right - dist_left) / wheel_separation_;

    // Intégration (approximation "arc" : on applique la rotation au milieu)
    double delta_x = d_center * std::cos(th_ + d_theta / 2.0);
    double delta_y = d_center * std::sin(th_ + d_theta / 2.0);

    x_  += delta_x;
    y_  += delta_y;
    th_ += d_theta;

    // --- 3) Vitesses ---
    double vx  = d_center / dt;
    double vth = d_theta  / dt;

    // --- 4) Quaternion ---
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, th_);

    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();

    // --- 5) Publication du message Odometry ---
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;

    // Pose
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = q_msg;

    // (Optionnel) Covariances simples
    for (int i = 0; i < 36; ++i) {
      odom.pose.covariance[i] = 0.0;
      odom.twist.covariance[i] = 0.0;
    }
    // Petite incertitude sur x, y, yaw
    odom.pose.covariance[0] = 0.01;   // x
    odom.pose.covariance[7] = 0.01;   // y
    odom.pose.covariance[35] = 0.05;  // yaw

    // Twist
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = vth;

    odom_pub_->publish(odom);

    // --- 6) TF odom -> base_link ---
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = current_time;
    odom_tf.header.frame_id = frame_id_;
    odom_tf.child_frame_id = child_frame_id_;

    odom_tf.transform.translation.x = x_;
    odom_tf.transform.translation.y = y_;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = q_msg;

    tf_broadcaster_->sendTransform(odom_tf);
  }

  // Paramètres
  double wheel_radius_;
  double wheel_separation_;
  int ticks_per_rev_;
  std::string frame_id_;
  std::string child_frame_id_;

  // Etat intégré
  double x_;
  double y_;
  double th_;

  bool first_msg_;
  rclcpp::Time last_time_;

  // ROS
  rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr motors_feedback_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WheelOdometryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
