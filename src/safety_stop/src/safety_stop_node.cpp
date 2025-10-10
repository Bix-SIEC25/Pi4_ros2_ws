#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/ultrasonic.hpp"

#include <algorithm>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

class SafetyStopNode : public rclcpp::Node {
public:
  SafetyStopNode() : rclcpp::Node("safety_stop_node"), steady_clock_(RCL_STEADY_TIME)
  {
    // Paramètres
    stop_dist_front_cm_ = declare_parameter<int>("stop_dist_front_cm", 10);
    stop_dist_rear_cm_  = declare_parameter<int>("stop_dist_rear_cm", 10);
    us_timeout_ms_      = declare_parameter<int>("us_timeout_ms", 200);
    cmd_timeout_ms_     = declare_parameter<int>("cmd_timeout_ms", 200);
    log_actions_        = declare_parameter<bool>("log_actions", true);

    // QoS
    auto qos_sensor = rclcpp::SensorDataQoS();  // for /us_data

    // Commands QoS: RELIABLE with depth 10 (to match can_tx_node)
    rclcpp::QoS qos_cmd(rclcpp::KeepLast(10));
    qos_cmd.reliable();

    // Publisher : commandes "filtrées"
    pub_safe_order_ = create_publisher<interfaces::msg::MotorsOrder>(
        "motors_order", qos_cmd);

    // Subscribers : US + commandes "raw"
    sub_us_ = create_subscription<interfaces::msg::Ultrasonic>(
        "us_data", qos_sensor,
        std::bind(&SafetyStopNode::onUltrasonic, this, _1));

    sub_raw_order_ = create_subscription<interfaces::msg::MotorsOrder>(
        "motors_order_raw", qos_cmd,
        std::bind(&SafetyStopNode::onMotorsOrderRaw, this, _1));




    // Timer santé + watchdog
    timer_health_ = create_wall_timer(50ms, std::bind(&SafetyStopNode::onTimer, this));

    last_us_time_   = now();
    last_cmd_time_  = steady_clock_.now(); // watchdog sur horloge monotone

    RCLCPP_INFO(get_logger(), "safety_stop_node READY (front=%d cm, rear=%d cm, us_to=%d ms, cmd_to=%d ms)", stop_dist_front_cm_, stop_dist_rear_cm_, us_timeout_ms_, cmd_timeout_ms_);
  }

private:
  // ---- Données ----
  rclcpp::Publisher<interfaces::msg::MotorsOrder>::SharedPtr pub_safe_order_;
  rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr sub_us_;
  rclcpp::Subscription<interfaces::msg::MotorsOrder>::SharedPtr sub_raw_order_;
  rclcpp::TimerBase::SharedPtr timer_health_;

  interfaces::msg::Ultrasonic last_us_;
  rclcpp::Time last_us_time_;
  bool have_us_ = false;

  // Watchdog & sécurité
  rclcpp::Clock steady_clock_;           // horloge monotone pour watchdog cmd
  rclcpp::Time last_cmd_time_;           // en steady time
  int stop_dist_front_cm_;
  int stop_dist_rear_cm_;
  int us_timeout_ms_;
  int cmd_timeout_ms_;
  bool log_actions_;

  // ---- Utils ----
  static inline int clamp_pwm(int v) { return std::max(0, std::min(100, v)); }

  // ---- Callbacks ----
  void onUltrasonic(const interfaces::msg::Ultrasonic &msg) {
    last_us_ = msg;
    last_us_time_ = now();   // ROS time (utile en sim)
    have_us_ = true;
  }

  void onMotorsOrderRaw(const interfaces::msg::MotorsOrder &raw) {
    last_cmd_time_ = steady_clock_.now();  // feed watchdog

    interfaces::msg::MotorsOrder safe = raw;

    // Détermination du sens plus robuste :
    // signe de la moyenne des écarts par rapport à 50
    const double dev_left  = static_cast<int>(raw.left_rear_pwm)  - 50.0;
    const double dev_right = static_cast<int>(raw.right_rear_pwm) - 50.0;
    const double dev_mean  = (dev_left + dev_right) / 2.0;
    const bool forward  = dev_mean > 0.5;     // marge pour éviter le flottement
    const bool backward = dev_mean < -0.5;

    const bool us_fresh = have_us_ && ((now() - last_us_time_).nanoseconds() / 1'000'000 <= us_timeout_ms_);

    bool must_stop = false;

    if (!us_fresh) {
      must_stop = true; // politique "fail-safe"
      if (log_actions_) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No fresh US data (>%d ms). Stopping for safety.", us_timeout_ms_);
    } else {
      if (forward) {
        const int fL = last_us_.front_left;
        const int fC = last_us_.front_center;
        const int fR = last_us_.front_right;
        if ((fL >= 0 && fL <= stop_dist_front_cm_) || (fC >= 0 && fC <= stop_dist_front_cm_) || (fR >= 0 && fR <= stop_dist_front_cm_)) {
          must_stop = true;
          if (log_actions_) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Safety STOP (forward): obstacle <= %d cm (FL=%d, FC=%d, FR=%d).", stop_dist_front_cm_, fL, fC, fR);
        }
      } else if (backward) {
        const int rL = last_us_.rear_left;
        const int rC = last_us_.rear_center;
        const int rR = last_us_.rear_right;
        if ((rL >= 0 && rL <= stop_dist_rear_cm_) || (rC >= 0 && rC <= stop_dist_rear_cm_) || (rR >= 0 && rR <= stop_dist_rear_cm_)) {
          must_stop = true;
          if (log_actions_) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Safety STOP (reverse): obstacle <= %d cm (RL=%d, RC=%d, RR=%d).", stop_dist_rear_cm_, rL, rC, rR);
        }
      }
    }

    if (must_stop) {
      safe.left_rear_pwm  = 50;
      safe.right_rear_pwm = 50;
      // steering laissé tel quel
    }

    // clamp de sécurité
    safe.left_rear_pwm  = clamp_pwm(safe.left_rear_pwm);
    safe.right_rear_pwm = clamp_pwm(safe.right_rear_pwm);

    pub_safe_order_->publish(safe);
  }

  void onTimer() {
    // 1) US manquants
    if (!have_us_) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No ultrasonic data received on /us_data yet.");

    // 2) Watchdog commandes
    const auto age_ms = (steady_clock_.now() - last_cmd_time_).nanoseconds() / 1'000'000;
    if (age_ms > cmd_timeout_ms_) {
      interfaces::msg::MotorsOrder stop;
      stop.left_rear_pwm  = 50;
      stop.right_rear_pwm = 50;
      // steering non touché
      pub_safe_order_->publish(stop);
      if (log_actions_) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Cmd timeout > %d ms (age=%ld ms): forcing STOP.", cmd_timeout_ms_, (long)age_ms);
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyStopNode>());
  rclcpp::shutdown();
  return 0;
}
