#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>
#include <cmath>

using controller_interface::ControllerInterface;

class F1TenthCommandsController : public ControllerInterface {
public:
  // Foxy: init 시그니처
  controller_interface::return_type init(const std::string & controller_name) override
  {
    auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK) {
        return ret;
    }

    auto node = get_node(); 
    // 안전 선언 헬퍼
    auto safe_declare = [&](const std::string & name, const rclcpp::ParameterValue & def) {
        if (!node->has_parameter(name)) {
        node->declare_parameter(name, def);
        }
    };

    // 🔧 여기서 전부 safe_declare 로 변경
    safe_declare("left_steer_joint",  rclcpp::ParameterValue(std::string("front_left_steer_joint")));
    safe_declare("right_steer_joint", rclcpp::ParameterValue(std::string("front_right_steer_joint")));
    safe_declare("left_wheel_joint",  rclcpp::ParameterValue(std::string("rear_left_wheel_joint")));
    safe_declare("right_wheel_joint", rclcpp::ParameterValue(std::string("rear_right_wheel_joint")));

    safe_declare("servo_center",        rclcpp::ParameterValue(0.5));
    safe_declare("steer_max_left_rad",  rclcpp::ParameterValue(0.349066));
    safe_declare("steer_max_right_rad", rclcpp::ParameterValue(0.349066));
    safe_declare("servo_axis_inverted", rclcpp::ParameterValue(false));
    safe_declare("erpm_per_radps",      rclcpp::ParameterValue(300.0));

    // 파라미터 읽기
    node->get_parameter("left_steer_joint",  left_steer_joint_);
    node->get_parameter("right_steer_joint", right_steer_joint_);
    node->get_parameter("left_wheel_joint",  left_wheel_joint_);
    node->get_parameter("right_wheel_joint", right_wheel_joint_);
    node->get_parameter("servo_center",        servo_center_);
    node->get_parameter("steer_max_left_rad",  steer_left_max_);
    node->get_parameter("steer_max_right_rad", steer_right_max_);
    node->get_parameter("servo_axis_inverted", servo_invert_);
    node->get_parameter("erpm_per_radps",      erpm_per_radps_);

    // 구독자 생성(foxy에선 init에서 만들어도 OK)
    sub_servo_ = node->create_subscription<std_msgs::msg::Float64>(
      "/commands/servo/position", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64::SharedPtr m){
        double v = m->data;
        if (v < 0.0) v = 0.0;
        if (v > 1.0) v = 1.0;
        servo_in_ = v;
      });

    sub_motor_ = node->create_subscription<std_msgs::msg::Float64>(
      "/commands/motor/speed", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64::SharedPtr m){
        erpm_in_ = m->data;
      });

    // 초기값
    servo_in_ = 0.5;
    erpm_in_  = 0.0;

    return controller_interface::return_type::OK;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    conf.names = {
      left_steer_joint_  + "/position",
      right_steer_joint_ + "/position",
      left_wheel_joint_  + "/velocity",
      right_wheel_joint_ + "/velocity"
    };
    return conf;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }

  // Foxy: update() 인자 없음
  controller_interface::return_type update() override
  {
    double pos   = servo_invert_ ? (1.0 - servo_in_) : servo_in_;
    const double c = clamp01(servo_center_);
    double delta = 0.0;

    if (pos < c && c > 0.0) {
      const double t = (pos - c) / c;           // [-1,0)
      delta = -std::abs(t) * steer_left_max_;   // 왼쪽 음수
    } else if (pos > c && c < 1.0) {
      const double t = (pos - c) / (1.0 - c);   // (0,1]
      delta = +t * steer_right_max_;            // 오른쪽 양수
    } else {
      delta = 0.0;
    }

    const double omega = (erpm_per_radps_ != 0.0) ? (erpm_in_ / erpm_per_radps_) : 0.0;

    // command_interfaces_ 순서는 command_interface_configuration() 정의 순서
    command_interfaces_[0].set_value(delta);  // left steer position
    command_interfaces_[1].set_value(delta);  // right steer position
    command_interfaces_[2].set_value(omega);  // left wheel velocity
    command_interfaces_[3].set_value(omega);  // right wheel velocity

    return controller_interface::return_type::OK;
  }

private:
  static double clamp01(double v){ return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }

  // 파라미터/상태
  std::string left_steer_joint_, right_steer_joint_, left_wheel_joint_, right_wheel_joint_;
  double servo_center_{0.5};
  double steer_left_max_{0.349066};
  double steer_right_max_{0.349066};
  bool   servo_invert_{false};
  double erpm_per_radps_{300.0};

  // 입력 버퍼
  double servo_in_{0.5};
  double erpm_in_{0.0};

  // 구독자
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_servo_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_motor_;
};

PLUGINLIB_EXPORT_CLASS(F1TenthCommandsController, controller_interface::ControllerInterface)
