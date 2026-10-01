#include <atomic>
#include <QPushButton>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/empty.hpp>

namespace ur3e_write_q
{
// Keep MoveIt's planning frame and stored plan; only change its preview renderer.
class QMotionPlanningDisplay : public moveit_rviz_plugin::MotionPlanningDisplay
{
public:
  void update(float wall_dt, float ros_dt) override
  {
    const int command = pending_.exchange(-1);
    if (command >= 0)
      setPreview(command == 1);
    MotionPlanningDisplay::update(wall_dt, ros_dt);
  }

protected:
  void onInitialize() override
  {
    MotionPlanningDisplay::onInitialize();
    auto* plan = frame_->findChild<QPushButton*>("plan_button");
    auto* execute = frame_->findChild<QPushButton*>("execute_button");
    if (!plan || !execute)
      throw std::runtime_error("Cannot locate MoveIt Plan/Execute buttons");
    // These signals run on the GUI thread, just like MoveIt's own handlers.
    QObject::connect(plan, &QPushButton::clicked, this, [this]() { setPreview(true); });
    QObject::connect(execute, &QPushButton::clicked, this, [this]() { setPreview(false); });
    auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    plan_sub_ = node->create_subscription<std_msgs::msg::Empty>(
        "/rviz/moveit/plan", rclcpp::SystemDefaultsQoS(),
        [this](std_msgs::msg::Empty::ConstSharedPtr) { pending_.store(1); });
    execute_sub_ = node->create_subscription<std_msgs::msg::Empty>(
        "/rviz/moveit/execute", rclcpp::SystemDefaultsQoS(),
        [this](std_msgs::msg::Empty::ConstSharedPtr) { pending_.store(0); });
  }

private:
  void setPreview(bool visible)
  {
    auto* path = subProp("Planned Path");
    path->subProp("Show Robot Visual")->setValue(visible);
    path->subProp("Show Robot Collision")->setValue(false);
    path->subProp("Show Trail")->setValue(false);
    if (!visible)
      dropVisualizedTrajectory();
    RCLCPP_INFO(rclcpp::get_logger("q_preview"), "Planned robot preview %s",
                visible ? "enabled" : "stopped and hidden; execution plan retained");
  }

  std::atomic<int> pending_{-1};
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr plan_sub_, execute_sub_;
};
}  // namespace ur3e_write_q

PLUGINLIB_EXPORT_CLASS(ur3e_write_q::QMotionPlanningDisplay, rviz_common::Display)
