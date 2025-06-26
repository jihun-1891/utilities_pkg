#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>                         // FSM state broadcast
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

/****************************************************************************************
 * 글로벌 상수 (튜닝 값)                                                               *
 * ------------------------------------------------------------------------------------ *
 *  ▸ 실전 중 현장 튜닝이 필요한 값들은 한눈에 보이도록 파일 상단에 모았습니다.          *
 *  ▸ ROS 파라미터 서버를 통하지 않고 하드코딩‑상수로만 사용합니다.                      *
 ****************************************************************************************/
constexpr double KP_XY            = 0.2;   // XY P‑gain
constexpr double KD_XY            = 0.05;  // XY D‑gain
constexpr double MAX_XY_VEL       = 0.3;   // [m/s] XY 속도 상한(Body)
constexpr double VZ_FAST          = 0.5;   // [m/s] FINE 단계 하강 속도 (+ = Down in NED)
constexpr double VZ_SLOW          = 0.02;   // [m/s] WAIT / HOLD / COARSE 하강 속도
constexpr double SEARCH_UP_VEL    = -0.1;  // [m/s] SEARCH 단계 상승 속도 (− = Up)
constexpr double CLOSE_ALT        = 0.15;  // [m]   저고도 임계 (마커 미검출 가능)
constexpr double LOST_SHORT       = 0.3;   // [s]   짧은 끊김 허용 길이
constexpr double LOST_LONG        = 0.7;   // [s]   저고도 강제 착륙용 끊김 길이
constexpr double HOLD_TIMEOUT     = 2.0;   // [s]   HOLD → SEARCH 전환 시간
constexpr double SEARCH_TIMEOUT   = 4.0;   // [s]   SEARCH 중단 시간
constexpr double K_YAW            = 0.2;   // [rad/s per rad] Yaw P‑gain
constexpr double YAW_MAX          = 0.5;   // [rad/s] Yaw 속도 상한
constexpr double YAW_TH           = 0.05;  // [rad]   Yaw 정렬 허용 오차 (≈3°)

/****************************************************************************************
 * ArucoLandingNode                                                                     *
 * ------------------------------------------------------------------------------------ *
 *  ‣ ArUco 마커(/x500/target_pose) 기반 자동 착륙 FSM                                   *
 *  ‣ FSM 상태: WAIT → YAW → COARSE → FINE (+ HOLD / SEARCH 안전 상태)                   *
 *  ‣ 모든 시간 계산은 STEADY 시간(시뮬레이션 안전) 사용                                *
 ****************************************************************************************/

class ArucoLandingNode : public rclcpp::Node
{
public:
  ArucoLandingNode()
  : Node("aruco_landing_node"),
    steady_clock_{RCL_STEADY_TIME}
  {
    /* QoS 및 통신 설정 */
    rclcpp::QoS pose_qos(rclcpp::KeepLast(10)); pose_qos.reliable();
    rclcpp::QoS sensor_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    sensor_qos.best_effort();

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/x500/target_pose", pose_qos,
      std::bind(&ArucoLandingNode::pose_cb, this, std::placeholders::_1));

    heading_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", sensor_qos,
      std::bind(&ArucoLandingNode::heading_cb, this, std::placeholders::_1));

    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/command/twist", 10);
    cmd_pub_   = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("~/state", 10);

    timer_ = create_wall_timer(50ms, std::bind(&ArucoLandingNode::timer_cb, this));

    RCLCPP_INFO(get_logger(), "ArucoLandingNode ready (hard‑coded gains mode)");
  }

private:
  /* ───────── FSM & Timing ───────── */
  enum class State { WAIT, YAW, COARSE, FINE, HOLD, SEARCH, DONE };
  State      state_{State::WAIT};
  State      prev_state_{State::WAIT};
  rclcpp::Clock steady_clock_;
  rclcpp::Time state_enter_;

  double since_state() { return (steady_clock_.now() - state_enter_).seconds(); }
  void transit(State s)
  {
    prev_state_  = state_;
    state_       = s;
    state_enter_ = steady_clock_.now();
    std_msgs::msg::String m; m.data = state_name(s); state_pub_->publish(m);
  }

  /* ───────── ROS Handles ───────── */
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr heading_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr             twist_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr         cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 state_pub_;
  rclcpp::TimerBase::SharedPtr                                        timer_;

  /* ───────── State Variables ───────── */
  bool         has_pose_{false};
  geometry_msgs::msg::PoseStamped latest_pose_;
  rclcpp::Time last_pose_time_{steady_clock_.now()};
  double       heading_rad_{0.0};
  double       prev_err_fwd_{0.0}, prev_err_left_{0.0};
  rclcpp::Time prev_err_time_{steady_clock_.now()};
  geometry_msgs::msg::Twist last_cmd_;

  /* ───────── Callbacks ───────── */
  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    latest_pose_    = *msg;
    has_pose_       = true;
    last_pose_time_ = steady_clock_.now();
  }
  void heading_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  { heading_rad_ = msg->heading; }

  /* ───────── Main Timer ───────── */
  void timer_cb()
  {
    double t_pose = (steady_clock_.now() - last_pose_time_).seconds();
    bool   pose_ok = has_pose_ && (t_pose < LOST_SHORT);

    /* 저고도‑무포즈 보호 착륙 */
    if(!pose_ok && has_pose_ && latest_pose_.pose.position.z < CLOSE_ALT && t_pose > LOST_LONG)
      return finish_landing("Pose lost near ground → force land");

    /* 초기 포즈 대기 */
    if(state_ == State::WAIT && !pose_ok) { publish_vel(0,0,VZ_SLOW,0); return; }

    /* --- 오차 계산 (pose_ok) --- */
    double err_fwd=0, err_left=0, yaw_err=0, alt=0;
    if(pose_ok) {
      tf2::Vector3 p_rf(latest_pose_.pose.position.x, latest_pose_.pose.position.y, latest_pose_.pose.position.z);
      tf2::Quaternion q_m2d; tf2::fromMsg(latest_pose_.pose.orientation, q_m2d);
      tf2::Vector3 p_body = tf2::quatRotate(q_m2d.inverse(), p_rf);
      tf2::Vector3 err_b = -p_body;
      err_fwd  = err_b.x();              // + forward
      err_left = err_b.y();              // + left  (FLU)
      yaw_err  = tf2::getYaw(q_m2d);
      if(yaw_err> M_PI) yaw_err-=2*M_PI; if(yaw_err<-M_PI) yaw_err+=2*M_PI;
      alt      = p_rf.z();               // + up (RFU)
    } else {
      twist_pub_->publish(last_cmd_);
    }

    /* --- FSM --- */
    switch(state_) {
      case State::WAIT:
        if(pose_ok) transit(State::YAW);
        break;

      case State::YAW: {
        double yaw_rate = std::clamp(-K_YAW*yaw_err, -YAW_MAX, YAW_MAX);
        publish_vel(0,0,VZ_SLOW,yaw_rate);
        if(fabs(yaw_err) < 0.09 && since_state()>0.3) transit(State::COARSE);
      } break;

      case State::COARSE: {
        double vx,vy; pid_xy(err_fwd,err_left,vx,vy,true);
        publish_vel(vx,vy,VZ_SLOW,std::clamp(-K_YAW*yaw_err,-YAW_MAX,YAW_MAX));
        if(hypot(err_fwd,err_left) < 0.15 && since_state() > 0.3)
          transit(State::FINE);
      } break;

      case State::FINE: {
        double vx,vy; pid_xy(err_fwd,err_left,vx,vy,false);
        publish_vel(vx,vy,VZ_FAST,std::clamp(-K_YAW*yaw_err,-YAW_MAX,YAW_MAX));
        if(hypot(err_fwd,err_left) < 0.05 && alt < 0.01)
          finish_landing("Landing complete");
      } break;

      case State::HOLD: {
        publish_vel(0,0,0,0);
        if(pose_ok)                   transit(prev_state_);
        else if(since_state() > HOLD_TIMEOUT) transit(State::SEARCH);
      } break;

      case State::SEARCH: {
        publish_vel(0,0,SEARCH_UP_VEL,0);
        if(pose_ok)                   transit(prev_state_);
        else if(since_state() > SEARCH_TIMEOUT)
          finish_landing("Marker lost too long → abort");
      } break;

      case State::DONE:
        return;
    }

    /* HOLD 조건 진입 */
    if(!pose_ok && state_!=State::HOLD && state_!=State::SEARCH && alt> CLOSE_ALT && t_pose> LOST_SHORT)
      transit(State::HOLD);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "[%s] h=%.2f z=%.2f yaw=%.1f° | vel=[%.2f,%.2f,%.2f] yaẇ=%.2f",
      state_name(state_), hypot(err_fwd,err_left), alt, yaw_err*180/M_PI,
      last_cmd_.linear.x,last_cmd_.linear.y,last_cmd_.linear.z,last_cmd_.angular.z);
  }

  /*──────── PID (Body FLU→FRD→NED) ───────*/
  void pid_xy(double err_fwd,double err_left,double &vx_ned,double &vy_ned,bool coarse)
  {
    double kp = coarse? KP_XY*0.7 : KP_XY;
    double kd = coarse? KD_XY*0.7 : KD_XY;
    double dt = std::max((steady_clock_.now()-prev_err_time_).seconds(),1e-3);
    double derr_f = (err_fwd  - prev_err_fwd_) / dt;
    double derr_l = (err_left - prev_err_left_)/ dt;
    prev_err_fwd_=err_fwd; prev_err_left_=err_left; prev_err_time_=steady_clock_.now();

    double vx_b = kp*err_fwd  + kd*derr_f;            // +X forward
    double vy_b = -(kp*err_left + kd*derr_l);         // +Y right (FLU +left → FRD +right)
    vx_b = std::clamp(vx_b,-MAX_XY_VEL,MAX_XY_VEL);
    vy_b = std::clamp(vy_b,-MAX_XY_VEL,MAX_XY_VEL);

    vx_ned =  vx_b * cos(heading_rad_) - vy_b * sin(heading_rad_);
    vy_ned =  vx_b * sin(heading_rad_) + vy_b * cos(heading_rad_);
  }

  void publish_vel(double vx,double vy,double vz,double yaw_rate)
  {
    last_cmd_.linear.x  = vx;
    last_cmd_.linear.y  = vy;
    last_cmd_.linear.z  = vz;         // +Z NED = Down
    last_cmd_.angular.z = yaw_rate;
    twist_pub_->publish(last_cmd_);
  }

  void finish_landing(const char* txt)
  {
    send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,0.0);
    RCLCPP_INFO(get_logger(),"%s",txt);
    state_ = State::DONE;
    rclcpp::shutdown();
  }

  void send_cmd(uint16_t cmd,float p1=0,float p2=0,float p3=0,float p4=0,float p5=0,float p6=0,float p7=0)
  {
    px4_msgs::msg::VehicleCommand m{};
    m.timestamp = steady_clock_.now().nanoseconds()/1000;
    m.command = cmd;
    m.param1=p1; m.param2=p2; m.param3=p3; m.param4=p4; m.param5=p5; m.param6=p6; m.param7=p7;
    m.target_system=1; m.target_component=1;
    m.source_system=1; m.source_component=1;
    m.from_external=true;
    cmd_pub_->publish(m);
  }

  const char* state_name(State s) const {
    switch(s){
      case State::WAIT:   return "WAIT";
      case State::YAW:    return "YAW_ALIGN";
      case State::COARSE: return "COARSE";
      case State::FINE:   return "FINE";
      case State::HOLD:   return "HOLD";
      case State::SEARCH: return "SEARCH";
      case State::DONE:   return "DONE";
    }
    return "?";
  }
};

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<ArucoLandingNode>());
  rclcpp::shutdown();
  return 0;
}
