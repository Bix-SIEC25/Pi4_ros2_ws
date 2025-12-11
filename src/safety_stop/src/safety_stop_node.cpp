#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/ultrasonic.hpp"
#include "interfaces/msg/log_entry.hpp"
#include <algorithm>
#include <chrono>
#include <limits>

using std::placeholders::_1;
using namespace std::chrono_literals;

class SafetyStopNode : public rclcpp::Node {
public:
  SafetyStopNode() : rclcpp::Node("safety_stop_node"), steady_clock_(RCL_STEADY_TIME)
  {
    // Paramètres distance
    stop_dist_front_cm_ = declare_parameter<int>("stop_dist_front_cm", 20);
    stop_dist_rear_cm_  = declare_parameter<int>("stop_dist_rear_cm", 20);
    slow_dist_front_cm_ = declare_parameter<int>("slow_dist_front_cm", 45);
    slow_dist_rear_cm_  = declare_parameter<int>("slow_dist_rear_cm", 45);

    us_timeout_ms_      = declare_parameter<int>("us_timeout_ms", 200);
    cmd_timeout_ms_     = declare_parameter<int>("cmd_timeout_ms", 200);
    log_actions_        = declare_parameter<bool>("log_actions", true);

    // Rampe asymétrique
    max_delta_pwm_up_   = declare_parameter<int>("max_delta_pwm_up_per_msg", 1);
    max_delta_pwm_down_ = declare_parameter<int>("max_delta_pwm_down_per_msg", 1);
    max_delta_pwm_up_   = std::clamp(max_delta_pwm_up_,   0, 100);
    max_delta_pwm_down_ = std::clamp(max_delta_pwm_down_, 0, 100);

    // QoS
    auto qos_sensor = rclcpp::SensorDataQoS();  // /us_data

    rclcpp::QoS qos_cmd(rclcpp::KeepLast(10));  // = can_tx_node
    qos_cmd.reliable();

    // Publisher : commandes "filtrées"
    pub_safe_order_ = create_publisher<interfaces::msg::MotorsOrder>(
        "motors_order", qos_cmd);

    // Publisher : logging stops to web interface
    pub_log_ = create_publisher<interfaces::msg::LogEntry>("/logger", 10);
    
    // Souscriptions
    sub_us_ = create_subscription<interfaces::msg::Ultrasonic>(
        "us_data", qos_sensor,
        std::bind(&SafetyStopNode::onUltrasonic, this, _1));

    sub_raw_order_ = create_subscription<interfaces::msg::MotorsOrder>(
        "motors_order_raw", qos_cmd,
        std::bind(&SafetyStopNode::onMotorsOrderRaw, this, _1));

    // Timer santé + watchdog
    timer_health_ = create_wall_timer(50ms, std::bind(&SafetyStopNode::onTimer, this));

    last_us_time_   = now();
    last_cmd_time_  = steady_clock_.now();

    prev_cmd_valid_ = false;

    RCLCPP_INFO(get_logger(),
        "safety_stop_node READY (front_stop=%d cm, rear_stop=%d cm, "
        "front_slow=%d cm, rear_slow=%d cm, us_to=%d ms, cmd_to=%d ms, "
        "dPWM_up=%d/msg, dPWM_down=%d/msg)",
        stop_dist_front_cm_, stop_dist_rear_cm_,
        slow_dist_front_cm_, slow_dist_rear_cm_,
        us_timeout_ms_, cmd_timeout_ms_,
        max_delta_pwm_up_, max_delta_pwm_down_);
  }

private:
  // ---- Données ----
  rclcpp::Publisher<interfaces::msg::MotorsOrder>::SharedPtr pub_safe_order_;
  rclcpp::Publisher<interfaces::msg::LogEntry>::SharedPtr pub_log_;
  rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr sub_us_;
  rclcpp::Subscription<interfaces::msg::MotorsOrder>::SharedPtr sub_raw_order_;
  rclcpp::TimerBase::SharedPtr timer_health_;

  interfaces::msg::Ultrasonic last_us_;
  rclcpp::Time last_us_time_;
  bool have_us_ = false;

  // Watchdog & sécurité
  rclcpp::Clock steady_clock_;
  rclcpp::Time last_cmd_time_;
  int stop_dist_front_cm_;
  int stop_dist_rear_cm_;
  int slow_dist_front_cm_;
  int slow_dist_rear_cm_;
  int us_timeout_ms_;
  int cmd_timeout_ms_;
  bool log_actions_;

  // Rampe
  int max_delta_pwm_up_;
  int max_delta_pwm_down_;
  interfaces::msg::MotorsOrder prev_cmd_;
  bool prev_cmd_valid_;

  // To avoid sending several log messages per stop:
  bool safety_stop_activated = false;

  // ---- Utils ----
  static inline int clamp_pwm(int v) { return std::max(0, std::min(100, v)); }

  const int int_max = std::numeric_limits<int>::max(); // used in fucntion smallest_us()

  
  enum us_direction_t{
    LEFT,
    MIDDLE,
    RIGHT,
  };

  //stores the readout value AND direction of an ultrasound measurement
  struct us_readout_t{
    int us_value;
    us_direction_t direction;
  };

  //From a us readout message and forward/backward direction indication, returns the value and direction (left/middle/right) of the smallest readout
  us_readout_t smallest_us(const interfaces::msg::Ultrasonic &msg, bool forward){
    us_direction_t direction;
    int retval;
    int left;
    int right;
    int middle;

    if (forward){
      left = msg.front_left;
      middle = msg.front_center;
      right = msg.front_right;
    }
    else{
      left = msg.rear_left;
      middle = msg.rear_center;
      right = msg.rear_right;
    }

    int smallest = int_max;

    if (left < smallest){
      retval = left;
      direction = LEFT;
    }
    if (middle < smallest){
      retval = middle;
      direction = MIDDLE;
    }
    if (right < smallest){
      retval = right;
      direction = RIGHT;
    }

    return {retval, direction};
  }
    

  // Clamp forward : on limite l'amplitude vers 100 (PWM > 50)
  int computeClampedPwmForward(int pwm_req, int d, int d_stop, int d_slow, bool *stopped /*used to indicate if the clamp function stopped the car*/) {
    pwm_req = clamp_pwm(pwm_req);

    if (d < 0) {
      // mesure bizarre → ne pas limiter plus que demandé
      return pwm_req;
    }

    if (d_slow <= d_stop) {
      // zone de ralentissement mal paramétrée → arrêt dur seulement
      if (d <= d_stop) return 50;
      return pwm_req;
    }

    if (d <= d_stop) {
      // zone d’arrêt : PWM = 50 (neutre)
	  *stopped = true;
      return 50;
    }

    if (d >= d_slow) {
      // loin : pas de limitation
      safety_stop_activated = false;
      return pwm_req;
    }

    // Interpolation linéaire entre 50 (à d_stop) et 100 (à d_slow)
    float t = static_cast<float>(d - d_stop) /
              static_cast<float>(d_slow - d_stop);
    int pwm_max = 50 + static_cast<int>(t * (100 - 50) + 0.5f);

    pwm_max = clamp_pwm(pwm_max);
    return std::min(pwm_req, pwm_max);
  }

  // Clamp backward : on limite l'amplitude vers 0 (PWM < 50)
  int computeClampedPwmBackward(int pwm_req, int d, int d_stop, int d_slow) {
    pwm_req = clamp_pwm(pwm_req);

    if (d < 0) {
      // mesure bizarre → ne pas limiter plus que demandé
      return pwm_req;
    }

    if (d_slow <= d_stop) {
      // zone de ralentissement mal paramétrée → arrêt dur seulement
      if (d <= d_stop) return 50;
      return pwm_req;
    }

    if (d <= d_stop) {
      // zone d’arrêt : PWM = 50 (neutre)
      if (not(safety_stop_activated)){
      std::string message = "safety_stop_node stopping car due to: [ULTRASOUND DETECTED OBJECT TOO CLOSE WHILE MOVING BACKWARD (limit= " + std::to_string(stop_dist_rear_cm_) +  "cm)]";
      web_logger(4,"safety_stop_node",message);
	}
      safety_stop_activated = true;
      return 50;}
	

    if (d >= d_slow) {
      // loin : pas de limitation
      safety_stop_activated = false;
      return pwm_req;
    }

    // On raisonne en "écart sous 50"
    // dev = 50 - pwm_req (dev ∈ [0..50])
    // - loin (d = d_slow) : dev_max = 50 → pwm_min = 0 (aucune limite)
    // - près (d = d_stop) : dev_max = 0  → pwm_min = 50 (neutre)
    float t = static_cast<float>(d - d_stop) /
              static_cast<float>(d_slow - d_stop);
    float dev_max = t * 50.0f;       // max écart autorisé sous 50
    int   pwm_min = 50 - static_cast<int>(dev_max + 0.5f);

    pwm_min = clamp_pwm(pwm_min);

    // En backward, PWM doit rester >= pwm_min (on interdit d'aller "trop loin" sous 50)
    if (pwm_req < pwm_min) {
      return pwm_min;
    }
    return pwm_req;
  }

  // Applique la rampe aux PWM arrière
  void applyRamp(interfaces::msg::MotorsOrder &cmd) {
    if (!prev_cmd_valid_) {
      cmd.left_rear_pwm  = clamp_pwm(cmd.left_rear_pwm);
      cmd.right_rear_pwm = clamp_pwm(cmd.right_rear_pwm);
      prev_cmd_ = cmd;
      prev_cmd_valid_ = true;
      return;
    }

    auto rampOne = [this](int prev, int target) -> int {
      target = clamp_pwm(target);
      prev   = clamp_pwm(prev);
      int diff = target - prev;

      if (diff > 0) {
        // ACCÉLÉRATION
        if (diff > max_delta_pwm_up_) diff = max_delta_pwm_up_;
      } else if (diff < 0) {
        // FREINAGE
        if (diff < -max_delta_pwm_down_) diff = -max_delta_pwm_down_;
      }

      return clamp_pwm(prev + diff);
    };

    cmd.left_rear_pwm  = rampOne(prev_cmd_.left_rear_pwm,  cmd.left_rear_pwm);
    cmd.right_rear_pwm = rampOne(prev_cmd_.right_rear_pwm, cmd.right_rear_pwm);

    prev_cmd_ = cmd;
    prev_cmd_valid_ = true;
  }

  // Publication centralisée
  void publishSafeOrder(const interfaces::msg::MotorsOrder &input,
                        bool bypass_ramp = false) {
    interfaces::msg::MotorsOrder out = input;

    if (!bypass_ramp) {
      applyRamp(out);  // rampe sur propulsion
    } else {
      out.left_rear_pwm  = clamp_pwm(out.left_rear_pwm);
      out.right_rear_pwm = clamp_pwm(out.right_rear_pwm);
      prev_cmd_ = out;
      prev_cmd_valid_ = true;
    }

    pub_safe_order_->publish(out);
  }

  //Publish log to web interface:
  void web_logger(int lvl, std::string sender, std::string message){
    auto msg = interfaces::msg::LogEntry();
    msg.level = lvl;
    msg.sender = sender;
    msg.message = message;

    pub_log_->publish(msg);
  }

  // ---- Callbacks ----
  void onUltrasonic(const interfaces::msg::Ultrasonic &msg) {
    last_us_ = msg;
    last_us_time_ = now();
    have_us_ = true;
  }

  void onMotorsOrderRaw(const interfaces::msg::MotorsOrder &raw) {
    last_cmd_time_ = steady_clock_.now();

    interfaces::msg::MotorsOrder safe = raw;

    // Détermination du sens (par rapport à 50)
    const double dev_left  = static_cast<int>(raw.left_rear_pwm)  - 50.0;
    const double dev_right = static_cast<int>(raw.right_rear_pwm) - 50.0;
    const double dev_mean  = (dev_left + dev_right) / 2.0;
    const bool forward  = dev_mean > 0.5;
    const bool backward = dev_mean < -0.5;

    const bool us_fresh =
        have_us_ &&
        ((now() - last_us_time_).nanoseconds() / 1'000'000 <= us_timeout_ms_);

    bool sensor_fail_stop = false;

    if (!us_fresh) {
      // Fail-safe : pas d'info fiable → STOP direct, sans rampe
      sensor_fail_stop = true;
      if (log_actions_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "No fresh US data (>%d ms). Hard STOP for safety.",
            us_timeout_ms_);
	if (not(safety_stop_activated)){
	web_logger(4,"safety_stop_node",
		   "safety_stop_node stopping car due to: [ULTRASOUND SENSOR DATA TIMEOUT]");
	  }
	safety_stop_activated = true;
      }
    } else {
      // US OK → clamp dynamique + rampe
      if (forward) {
		us_readout_t small_us = smallest_us(last_us_, /*forwards=*/true);

		bool stopped = false;
        int l = computeClampedPwmForward(safe.left_rear_pwm,
                                         small_us.us_value,
                                         stop_dist_front_cm_,
                                         slow_dist_front_cm_,
										 &stopped);
        int r = computeClampedPwmForward(safe.right_rear_pwm,
                                         small_us.us_value,
                                         stop_dist_front_cm_,
                                         slow_dist_front_cm_,
										 &stopped);
        safe.left_rear_pwm  = l;
        safe.right_rear_pwm = r;

		std::string dir_message;
		if (stopped){
				switch (small_us.direction){
						case LEFT:
								dir_message = "LEFT";
								break;
						case MIDDLE:
								dir_message = "MIDDLE";
								break;
						case RIGHT:
								dir_message = "RIGHT";
								break;
				}

				if (log_actions_ && not(safety_stop_activated)){
						web_logger(4,"safety_stop_node","safety_stop_node stopping car due to: [ULTRASOUND FRONT " + dir_message + " DETECTED OBJECT TOO CLOSE WHILE MOVING FORWARD (limit= " + std::to_string(stop_dist_rear_cm_) + "cm)]" );
				}
				safety_stop_activated = true;
		}

        if (log_actions_) {
          RCLCPP_DEBUG(get_logger(),
                       "Forward: d=%d cm, pwm_req=(%d,%d) -> clamp=(%d,%d)",
                       small_us.us_value,
                       raw.left_rear_pwm, raw.right_rear_pwm,
                       safe.left_rear_pwm, safe.right_rear_pwm);
        }
      } else if (backward) {
        int d = std::min({last_us_.rear_left,
                          last_us_.rear_center,
                          last_us_.rear_right});

        int l = computeClampedPwmBackward(safe.left_rear_pwm,
                                          d,
                                          stop_dist_rear_cm_,
                                          slow_dist_rear_cm_);
        int r = computeClampedPwmBackward(safe.right_rear_pwm,
                                          d,
                                          stop_dist_rear_cm_,
                                          slow_dist_rear_cm_);
        safe.left_rear_pwm  = l;
        safe.right_rear_pwm = r;

        if (log_actions_) {
          RCLCPP_DEBUG(get_logger(),
                       "Backward: d=%d cm, pwm_req=(%d,%d) -> clamp=(%d,%d)",
                       d,
                       raw.left_rear_pwm, raw.right_rear_pwm,
                       safe.left_rear_pwm, safe.right_rear_pwm);
        }
      }
    }

    if (sensor_fail_stop) {
      // Hard stop (capteur mort) : 50 direct, pas de rampe
      safe.left_rear_pwm  = 50;
      safe.right_rear_pwm = 50;
      publishSafeOrder(safe, /*bypass_ramp=*/true);
    } else {
      // Cas normal : clamp distance + rampe (accel/freinage)
      publishSafeOrder(safe, /*bypass_ramp=*/false);
    }
  }

  void onTimer() {
    if (!have_us_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "No ultrasonic data received on /us_data yet.");
    }

    // Watchdog commandes
    const auto age_ms =
        (steady_clock_.now() - last_cmd_time_).nanoseconds() / 1'000'000;
    if (age_ms > cmd_timeout_ms_) {
      interfaces::msg::MotorsOrder stop;
      stop.left_rear_pwm  = 50;
      stop.right_rear_pwm = 50;
      publishSafeOrder(stop, /*bypass_ramp=*/true);

      if (log_actions_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Cmd timeout > %d ms (age=%ld ms): forcing hard STOP.",
            cmd_timeout_ms_, (long)age_ms);
	if (not(safety_stop_activated)){
	    web_logger(4,"safety_stop_node",
		       "safety_stop_node stopping car due to: [WATCHDOG TIMEOUT]");
	  }
	safety_stop_activated = true;
	
      }
    }
  }

};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyStopNode>());
  rclcpp::shutdown();
  return 0;
}
