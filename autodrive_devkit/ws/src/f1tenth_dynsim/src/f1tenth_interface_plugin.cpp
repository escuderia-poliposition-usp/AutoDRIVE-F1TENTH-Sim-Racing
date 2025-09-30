// file: src/f1tenth_interface_plugin.cpp
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <string>
#include <memory>
#include <algorithm>

namespace f1tenth_dynsim
{

// Plugin "Dynamic-Actuator":
//  - Subscrive em steer (rad), throttle (%), brake (%)
//  - Aplica: posição nas juntas de esterço dianteiras; velocidade nas rodas traseiras
//  - Conversão simples: v_target = k_speed * throttle% * (1 - brake%)
//    w_target (rad/s) = v_target / wheel_radius

class F1tenthInterfacePlugin : public gazebo::ModelPlugin
{
public:
  F1tenthInterfacePlugin() = default;
  ~F1tenthInterfacePlugin() override = default;

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_ = gazebo_ros::Node::Get(sdf);

    // parâmetros
    steer_fl_name_ = sdf->HasElement("steer_fl_joint") ? sdf->Get<std::string>("steer_fl_joint") : "joint_steer_front_left";
    steer_fr_name_ = sdf->HasElement("steer_fr_joint") ? sdf->Get<std::string>("steer_fr_joint") : "joint_steer_front_right";
    wheel_rl_name_ = sdf->HasElement("wheel_rl_joint") ? sdf->Get<std::string>("wheel_rl_joint") : "joint_wheel_rear_left";
    wheel_rr_name_ = sdf->HasElement("wheel_rr_joint") ? sdf->Get<std::string>("wheel_rr_joint") : "joint_wheel_rear_right";

    steer_topic_    = sdf->HasElement("steer_topic")    ? sdf->Get<std::string>("steer_topic")    : "/autodrive/f1tenth_1/steering_command";
    throttle_topic_ = sdf->HasElement("throttle_topic") ? sdf->Get<std::string>("throttle_topic") : "/autodrive/f1tenth_1/throttle_command";
    brake_topic_    = sdf->HasElement("brake_topic")    ? sdf->Get<std::string>("brake_topic")    : "/autodrive/f1tenth_1/brake_command";

    k_speed_        = sdf->HasElement("k_speed")        ? sdf->Get<double>("k_speed") : 2.0;            // m/s @100% thr
    wheel_radius_   = sdf->HasElement("wheel_radius")   ? sdf->Get<double>("wheel_radius") : 0.034;     // m
    steer_limit_rad_= sdf->HasElement("steer_limit_rad")? sdf->Get<double>("steer_limit_rad"): (40.0 * M_PI/180.0);

    auto jfl = model_->GetJoint(steer_fl_name_);
    auto jfr = model_->GetJoint(steer_fr_name_);
    auto jrl = model_->GetJoint(wheel_rl_name_);
    auto jrr = model_->GetJoint(wheel_rr_name_);
    if (!jfl || !jfr || !jrl || !jrr) {
      RCLCPP_ERROR(node_->get_logger(), "Junta não encontrada (fl=%s, fr=%s, rl=%s, rr=%s)",
        steer_fl_name_.c_str(), steer_fr_name_.c_str(), wheel_rl_name_.c_str(), wheel_rr_name_.c_str());
      return;
    }
    steer_fl_ = jfl; steer_fr_ = jfr; wheel_rl_ = jrl; wheel_rr_ = jrr;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    sub_steer_ = node_->create_subscription<std_msgs::msg::Float32>(
      steer_topic_, qos, std::bind(&F1tenthInterfacePlugin::OnSteer, this, std::placeholders::_1));
    sub_throttle_ = node_->create_subscription<std_msgs::msg::Float32>(
      throttle_topic_, qos, std::bind(&F1tenthInterfacePlugin::OnThrottle, this, std::placeholders::_1));
    sub_brake_ = node_->create_subscription<std_msgs::msg::Float32>(
      brake_topic_, qos, std::bind(&F1tenthInterfacePlugin::OnBrake, this, std::placeholders::_1));

    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&F1tenthInterfacePlugin::OnUpdate, this));

    RCLCPP_INFO(node_->get_logger(), "f1tenth_interface_plugin carregado. Tópicos: %s, %s, %s",
      steer_topic_.c_str(), throttle_topic_.c_str(), brake_topic_.c_str());
  }

private:
  void OnSteer(const std_msgs::msg::Float32::SharedPtr msg)
  {
    steer_cmd_ = std::clamp<double>(msg->data, -steer_limit_rad_, steer_limit_rad_);
  }
  void OnThrottle(const std_msgs::msg::Float32::SharedPtr msg)
  {
    throttle_ = std::clamp<double>(msg->data, 0.0, 100.0);
  }
  void OnBrake(const std_msgs::msg::Float32::SharedPtr msg)
  {
    brake_ = std::clamp<double>(msg->data, 0.0, 100.0);
  }

  void OnUpdate()
  {
    if (!steer_fl_ || !steer_fr_ || !wheel_rl_ || !wheel_rr_) return;

    // esterço (posição)
    steer_fl_->SetPosition(0, steer_cmd_, true);
    steer_fr_->SetPosition(0, steer_cmd_, true);

    // velocidade desejada a partir de thr/brake
    const double v_throttle = k_speed_ * (throttle_ / 100.0);
    const double v = std::max(0.0, v_throttle * (1.0 - brake_ / 100.0));
    const double w = (wheel_radius_ > 1e-6) ? (v / wheel_radius_) : 0.0;

    wheel_rl_->SetVelocity(0, w);
    wheel_rr_->SetVelocity(0, w);
  }

private:
  gazebo::physics::ModelPtr model_;
  gazebo::event::ConnectionPtr update_conn_;
  gazebo_ros::Node::SharedPtr node_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_steer_, sub_throttle_, sub_brake_;

  std::string steer_fl_name_, steer_fr_name_, wheel_rl_name_, wheel_rr_name_;
  gazebo::physics::JointPtr steer_fl_, steer_fr_, wheel_rl_, wheel_rr_;

  std::string steer_topic_, throttle_topic_, brake_topic_;

  double steer_cmd_{0.0};
  double throttle_{0.0};
  double brake_{0.0};

  double k_speed_{2.0};
  double wheel_radius_{0.034};
  double steer_limit_rad_{40.0 * M_PI/180.0};
};

GZ_REGISTER_MODEL_PLUGIN(F1tenthInterfacePlugin)

} // namespace f1tenth_dynsim
