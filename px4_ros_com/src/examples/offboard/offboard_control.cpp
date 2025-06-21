// ✅ [1] offboard_control_node.cpp
// 위치 제어나 속도 제어를 외부 명령에 따라 전환하며 동작

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
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
    gimbal_pitch_sub_ = create_subscription<std_msgs::msg::Float32>("/gimbal_pitch_degree", 10, std::bind(&OffboardControl::gimbal_callback, this, _1));    
    land_sub_ = create_subscription<std_msgs::msg::Bool>("/command/land", 10, std::bind(&OffboardControl::land_callback, this, _1));
    land_detected_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected", 10, std::bind(&OffboardControl::land_detected_callback, this, _1));
    disarm_sub_ = create_subscription<std_msgs::msg::Bool>("/command/disarm", 10, std::bind(&OffboardControl::disarm_callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&OffboardControl::timer_callback, this));
  }

private:
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gimbal_pitch_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr land_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr disarm_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  ControlMode mode_ = ControlMode::POSITION;
  TrajectorySetpoint setpoint_{};
  int setpoint_counter_ = 0;

  const uint8_t MY_SYSID  = 46;   // 비행 컨트롤러와 동일
  const uint8_t MY_COMPID = 47;  // USER1
  const uint8_t TARGET_SYSID  = 1;   
  const uint8_t TARGET_COMPID = 1;  
  const uint8_t FLAG_GIMBAL = 12;

  bool   target_command_ = false;   // setpoint가 최소 한 번은 들어왔는가?
  bool   armed_          = false;   // 이미 arm 했는가?
  bool   landed_         = false;
  bool   landmode_       = false;

  void timer_callback()
  {
    publish_offboard_control_mode();
    trajectory_setpoint_pub_->publish(setpoint_);

    /* 아직 setpoint를 받은 적이 없다면 단순 송출만 */
    if (!target_command_)
    {
      RCLCPP_INFO(get_logger(), "Waiting for target command.");
      return;
    }

    /* pose를 받은 뒤 10회(≈1 s) 동안 연속 송출해야 PX4가 Offboard를 허용 */
    if (!armed_ || landed_ || !landmode_) {
      if (++setpoint_counter_ >= 10) {
        /* ① OFFBOARD 모드 설정  ② ARM */
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        armed_ = true;
        RCLCPP_INFO(get_logger(), "Set OFFBOARD mode & ARM (after pose received)");
      }
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

    target_command_   = true;
  }
  void gimbal_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    send_gimbal_pitch(static_cast<float>(msg->data));
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

    target_command_   = true;
    setpoint_counter_ = 0;
  }

  void publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = (mode_ == ControlMode::POSITION);
    msg.velocity = (mode_ == ControlMode::VELOCITY);
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

  void send_gimbal_pitch(float pitch_deg)
  {

    take_gimbal_control();
    /* 2) 피치 명령 */
    auto now_us = this->get_clock()->now().nanoseconds() / 1000;

    VehicleCommand cmd{};
    cmd.timestamp          = now_us;
    cmd.command            = VehicleCommand::VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW; // 1000
    cmd.param1             = pitch_deg;   // Pitch (+위, -아래)
    cmd.param2             = 0.0f;         // Yaw 유지
    cmd.param3             = NAN;          // rate
    cmd.param4             = NAN;        // rate
    cmd.param5             = FLAG_GIMBAL;           // follow(0) or lock(16)
    cmd.param7             = 1;           // gimbal_device_id
    cmd.target_system      = TARGET_SYSID;
    cmd.target_component   = TARGET_COMPID;
    cmd.source_system      = MY_SYSID;
    cmd.source_component   = MY_COMPID;   // ★ 반드시 동일
    cmd.from_external      = true;
    vehicle_command_pub_->publish(cmd);
  }

  void take_gimbal_control()
  {
    VehicleCommand cmd{};
    cmd.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
    cmd.command          = VehicleCommand::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;   // 1001

    /* ── 핵심: -2 를 넣으면 PX4가 msg 의 sys/comp id 그대로 채움 ── */
    cmd.param1 = MY_SYSID;          
    cmd.param2 = MY_COMPID;          
    cmd.param3 = cmd.param4 = 0;     // secondary 없음
    cmd.param5 = FLAG_GIMBAL;                   // flags
    cmd.param7 = 1;                   // gimbal device id

    cmd.target_system      = TARGET_SYSID;       // PX4
    cmd.target_component   = TARGET_COMPID;       // MAV_COMP_ID_AUTOPILOT
    cmd.source_system      = MY_SYSID;
    cmd.source_component   = MY_COMPID;
    cmd.from_external      = true;

    vehicle_command_pub_->publish(cmd);
    RCLCPP_INFO(get_logger(), ">>> Sent CONFIGURE(1001) – take primary control");
  }

  void land_callback(const std_msgs::msg::Bool::SharedPtr /*unused*/)
  {
    RCLCPP_INFO(get_logger(), "Land request received → sending NAV_LAND");

    /* ① 필요하다면 OFFBOARD 해제 후 AUTO.LAND 로 전환
       (PX4는 NAV_LAND만으로도 착륙을 시작하지만,
        확실히 하려면 SET_MODE로 AUTO.LAND(4) 전환 → 아래 한 줄 주석 해제)
    */
    landmode_ = true;
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4);

    /* ② NAV_LAND 지령 전송 (MAV_CMD_NAV_LAND = 21) */
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);

    /* 선택) 착륙 완료 후 자동 해제(Disarm)는 PX4가 알아서 수행하므로 생략 */
  }
  
  void land_detected_callback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
  {
    if (msg->landed)
    {
      landed_ = true;
      landmode_ = false;
    }
  }

  void disarm_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {                       // true 면 Disarm
      RCLCPP_INFO(get_logger(), "DISARM requested → sending command 400");
      publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,0.0f, 21196.0f);       // param2에 21196.0f는 시뮬레이션 대회를 위한 강제 disarm 입니다. 하늘에서 명령하면 프롭 멈춰서 떨어지니 실제로는 사용하지 마세요 -조교-
      armed_ = false;
      target_command_ = false;

    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControl>());
  rclcpp::shutdown();
  return 0;
}
