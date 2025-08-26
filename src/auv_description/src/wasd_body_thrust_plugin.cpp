#include <memory>
#include <string>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

using gz::sim::Model;
using gz::sim::Link;
using gz::sim::Entity;
using gz::sim::UpdateInfo;
using gz::sim::EntityComponentManager;

namespace auve1
{
class WasdBodyWrenchPlugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 EntityComponentManager &ecm,
                 gz::sim::EventManager &) override
  {
    model_ = Model(entity);
    if (!model_.Valid(ecm)) {
      throw std::runtime_error("WasdBodyWrenchPlugin: model invalid");
    }

    // Parameters
    link_name_ = sdf && sdf->HasElement("link_name") ?
                 sdf->Get<std::string>("link_name") : "base_link";

    // Topics (absolute recommended)
    force_topic_  = (sdf && sdf->HasElement("force_topic"))  ? sdf->Get<std::string>("force_topic")  : "/auve1/force_body";
    torque_topic_ = (sdf && sdf->HasElement("torque_topic")) ? sdf->Get<std::string>("torque_topic") : "/auve1/torque_body";

    // Hold behavior: -1 => hold forever, else ms window
    hold_ms_ = -1;
    if (sdf && sdf->HasElement("hold_ms")) hold_ms_ = sdf->Get<int>("hold_ms");

    // Optional scaling
    force_scale_  = (sdf && sdf->HasElement("force_scale"))  ? sdf->Get<double>("force_scale")  : 1.0;
    torque_scale_ = (sdf && sdf->HasElement("torque_scale")) ? sdf->Get<double>("torque_scale") : 1.0;

    // Link resolve
    linkEntity_ = model_.LinkByName(ecm, link_name_);
    if (linkEntity_ == gz::sim::kNullEntity) {
      throw std::runtime_error("WasdBodyWrenchPlugin: link '" + link_name_ + "' not found");
    }
    link_ = Link(linkEntity_);

    // ROS 2 node + subscriptions
    rclcpp::InitOptions opts;
    opts.shutdown_on_signal = false;
    if (!rclcpp::ok()) rclcpp::init(0, nullptr, opts);

    node_ = std::make_shared<rclcpp::Node>("wasd_body_wrench_plugin");
    clock_type_ = node_->get_clock()->get_clock_type();
    last_force_time_  = rclcpp::Time(0,0,clock_type_);
    last_torque_time_ = rclcpp::Time(0,0,clock_type_);

    sub_force_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
      force_topic_, 10,
      [this](geometry_msgs::msg::Vector3::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_);
        cmd_force_body_.Set(msg->x * force_scale_, msg->y * force_scale_, msg->z * force_scale_);
        last_force_time_ = node_->now();
        have_force_ = true;
      });

    sub_torque_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
      torque_topic_, 10,
      [this](geometry_msgs::msg::Vector3::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_);
        cmd_torque_body_.Set(msg->x * torque_scale_, msg->y * torque_scale_, msg->z * torque_scale_);
        last_torque_time_ = node_->now();
        have_torque_ = true;
      });

    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_);

    spinner_ = std::thread([this]{
      rclcpp::Rate r(300.0);
      while (rclcpp::ok()) {
        exec_->spin_some();
        r.sleep();
      }
    });

    RCLCPP_INFO(node_->get_logger(),
      "WasdBodyWrenchPlugin: link='%s', force_topic='%s', torque_topic='%s', hold_ms=%d, scales=(%.3f, %.3f)",
      link_name_.c_str(), force_topic_.c_str(), torque_topic_.c_str(), hold_ms_, force_scale_, torque_scale_);
  }

  void PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm) override
  {
    if (info.paused) return;

    if (!velocity_enabled_)
    {
      link_.EnableVelocityChecks(ecm, true);
      velocity_enabled_ = true;
    }

    gz::math::Vector3d f_body(0,0,0), tau_body(0,0,0);
    bool use_force = false, use_torque = false;

    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (have_force_) {
        if (hold_ms_ < 0) { use_force = true; f_body = cmd_force_body_; }
        else {
          auto now = rclcpp::Time(node_->now().nanoseconds(), clock_type_);
          if ((now - last_force_time_) < rclcpp::Duration::from_seconds(hold_ms_ / 1000.0))
            { use_force = true; f_body = cmd_force_body_; }
        }
      }
      if (have_torque_) {
        if (hold_ms_ < 0) { use_torque = true; tau_body = cmd_torque_body_; }
        else {
          auto now = rclcpp::Time(node_->now().nanoseconds(), clock_type_);
          if ((now - last_torque_time_) < rclcpp::Duration::from_seconds(hold_ms_ / 1000.0))
            { use_torque = true; tau_body = cmd_torque_body_; }
        }
      }
    }

    if (!use_force && !use_torque) return;

    auto pose = gz::sim::worldPose(linkEntity_, ecm);
    gz::math::Vector3d f_world   = use_force  ? pose.Rot().RotateVector(f_body)   : gz::math::Vector3d::Zero;
    gz::math::Vector3d tau_world = use_torque ? pose.Rot().RotateVector(tau_body) : gz::math::Vector3d::Zero;

    // Apply both force and torque in world frame at CoM
    link_.AddWorldWrench(ecm, f_world, tau_world);

    // (Optional) throttle debug
    if (++dbg_counter_ % 60 == 0 && node_) {
      RCLCPP_INFO(node_->get_logger(),
        "[WRENCH] F[%.1f %.1f %.1f]  Tau[%.1f %.1f %.1f]",
        f_world.X(), f_world.Y(), f_world.Z(), tau_world.X(), tau_world.Y(), tau_world.Z());
    }
  }

  ~WasdBodyWrenchPlugin() override
  {
    if (exec_) exec_->cancel();
    if (spinner_.joinable()) spinner_.join();
    if (rclcpp::ok()) rclcpp::shutdown();
  }

private:
  Model model_;
  std::string link_name_;
  std::string force_topic_;
  std::string torque_topic_;
  int    hold_ms_{-1};
  double force_scale_{1.0};
  double torque_scale_{1.0};

  Entity linkEntity_{gz::sim::kNullEntity};
  Link link_{gz::sim::kNullEntity};

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_force_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_torque_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::thread spinner_;
  std::mutex mtx_;

  gz::math::Vector3d cmd_force_body_{0,0,0};
  gz::math::Vector3d cmd_torque_body_{0,0,0};
  rclcpp::Time last_force_time_;
  rclcpp::Time last_torque_time_;
  rcl_clock_type_t clock_type_;

  bool have_force_{false};
  bool have_torque_{false};
  bool velocity_enabled_{false};
  int dbg_counter_{0};
};
}  // namespace auve1

GZ_ADD_PLUGIN(auve1::WasdBodyWrenchPlugin,
              gz::sim::System,
              auve1::WasdBodyWrenchPlugin::ISystemConfigure,
              auve1::WasdBodyWrenchPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(auve1::WasdBodyWrenchPlugin, "auve1::WasdBodyWrenchPlugin")
GZ_ADD_PLUGIN_ALIAS(auve1::WasdBodyWrenchPlugin, "wasd_body_wrench_plugin")

// add this extra alias so your world/URDF can request it:
GZ_ADD_PLUGIN_ALIAS(auve1::WasdBodyWrenchPlugin, "wasd_body_thrust_plugin")
