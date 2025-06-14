// ✅ [1] offboard_control_node.cpp
// 위치 제어나 속도 제어를 외부 명령에 따라 전환하며 동작

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h> 
#include <rclcpp/rclcpp.hpp>
#include <string>

using std::placeholders::_1;
using namespace px4_msgs::msg;

enum class ControlMode { POSITION, VELOCITY };

class OffboardControl : public rclcpp::Node
{
public:
  OffboardControl() : Node("offboard_control_node") {
    offboard_control_mode_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/command/pose", 10, std::bind(&OffboardControl::pose_callback, this, _1));
    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>("/command/twist", 10, std::bind(&OffboardControl::twist_callback, this, _1));

    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&OffboardControl::timer_callback, this));
  }

private:
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  ControlMode mode_ = ControlMode::POSITION;
  TrajectorySetpoint setpoint_{};
  int setpoint_counter_ = 0;

  void timer_callback() 
  {
    publish_offboard_control_mode();
    trajectory_setpoint_pub_->publish(setpoint_);

    if (setpoint_counter_ < 10) {
      setpoint_counter_++;
      return;
    }else if (setpoint_counter_ > 12)
    {
      return;
    }
    

    if (setpoint_counter_ == 10) {
      publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);  // PX4_CUSTOM_MAIN_MODE_OFFBOARD
      publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);  // Arm
      // publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, -3.0); // optional: 이륙 (z는 NED 기준)
      RCLCPP_INFO(this->get_logger(), "Sent offboard + arm");
    }

    setpoint_counter_++;
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    mode_ = ControlMode::POSITION;
    setpoint_.position[0] = msg->pose.position.x;
    setpoint_.position[1] = msg->pose.position.y;
    setpoint_.position[2] = msg->pose.position.z;

    double yaw = tf2::getYaw(msg->pose.orientation);
    setpoint_.yaw = yaw;
  }

  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    mode_ = ControlMode::VELOCITY;
    setpoint_.position[0] = NAN;
    setpoint_.position[1] = NAN;
    setpoint_.position[2] = NAN;
    setpoint_.velocity[0] = msg->linear.x;
    setpoint_.velocity[1] = msg->linear.y;
    setpoint_.velocity[2] = msg->linear.z;
    setpoint_.yaw = NAN;
    setpoint_.yawspeed = msg->angular.z;
  }

  void publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    msg.position = (mode_ == ControlMode::POSITION);
    msg.velocity = (mode_ == ControlMode::VELOCITY);
    offboard_control_mode_pub_->publish(msg);
  }

  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
  {
    VehicleCommand msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControl>());
  rclcpp::shutdown();
  return 0;
}
