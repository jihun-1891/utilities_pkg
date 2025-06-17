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

    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&OffboardControl::timer_callback, this));
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

  bool   target_command  = false;   // setpoint가 최소 한 번은 들어왔는가?
  bool   armed_          = false;   // 이미 arm 했는가?

  void timer_callback()
  {
    publish_offboard_control_mode();
    trajectory_setpoint_pub_->publish(setpoint_);

    /* 아직 setpoint를 받은 적이 없다면 단순 송출만 */
    if (!target_command)
    {
      RCLCPP_INFO(get_logger(), "Waiting for target command.");
      return;
    }

    /* pose를 받은 뒤 10회(≈1 s) 동안 연속 송출해야 PX4가 Offboard를 허용 */
    if (!armed_) {
      if (++setpoint_counter_ >= 10) {
        /* ① OFFBOARD 모드 설정  ② ARM */
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        armed_ = true;
        RCLCPP_INFO(get_logger(), "Set OFFBOARD mode & ARM (after pose received)");
      }
    }
    // stop the counter after reaching 11
    if (setpoint_counter_ < 11) {
      setpoint_counter_++;
    }
  } 

  /* Pose 콜백: setpoint 저장 + 플래그 ON + 카운터 초기화 */
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    mode_ = ControlMode::POSITION;
    setpoint_.position[0] = msg->pose.position.x;
    setpoint_.position[1] = msg->pose.position.y;
    setpoint_.position[2] = msg->pose.position.z;
    setpoint_.yaw         = tf2::getYaw(msg->pose.orientation);
    RCLCPP_INFO(get_logger(), "Target pose arrived. X: %f, Y: %f, Z: %f, Yaw: %f", 
                                setpoint_.position[0],setpoint_.position[1],setpoint_.position[2],setpoint_.yaw);

    target_command   = true;
  }

  /* Twist 콜백도 동일하게 플래그·카운터 처리 */
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    mode_ = ControlMode::VELOCITY;
    setpoint_.position[0] = NAN;
    setpoint_.position[1] = NAN;
    setpoint_.position[2] = NAN;
    setpoint_.velocity[0] = msg->linear.x;
    setpoint_.velocity[1] = msg->linear.y;
    setpoint_.velocity[2] = msg->linear.z;
    setpoint_.yaw         = NAN;
    setpoint_.yawspeed    = msg->angular.z;

    target_command   = true;
    setpoint_counter_ = 0;
  }

  void publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = (mode_ == ControlMode::POSITION);
    msg.velocity = (mode_ == ControlMode::VELOCITY);
    offboard_control_mode_pub_->publish(msg);
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
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
