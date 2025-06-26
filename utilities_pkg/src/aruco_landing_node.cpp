#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <cmath>
using namespace std::chrono_literals;

/******************** 튜닝 상수 (모두 +Z=Down, +Y=NED Right) ************************/
constexpr double KP_XY           = 0.1;   // XY P gain
constexpr double KD_XY           = 0.01;  // XY D gain
constexpr double MAX_XY_VEL      = 0.4;   // [m/s] XY 속도 제한
constexpr double VZ_FAST         = 0.5;   // [m/s] 본격 하강 속도 (+ = 아래)
constexpr double VZ_SLOW         = 0.2;   // [m/s] 초기·HOLD 단계 하강 속도
constexpr double SEARCH_UP_VEL   = -0.1;  // [m/s] SEARCH 상승 (- = 위)
constexpr double CLOSE_ALT       = 0.15;  // [m]   저고도 임계
constexpr double LOST_SHORT      = 2.0;   // [s]   짧은 끊김 허용 시간
constexpr double LOST_LONG       = 3.0;   // [s]   저고도 강제 착륙 끊김
constexpr double HOLD_TIMEOUT    = 2.0;   // [s]   HOLD 유지 시간
constexpr double SEARCH_TIMEOUT  = 10.0;   // [s]   SEARCH 중단 시간
constexpr double COARSE_THRESH   = 0.15;  // [m]   COARSE → FINE 전환
constexpr double FINE_THRESH     = 0.05;  // [m]   착륙 허용 수평 오차
/***********************************************************************************/

class ArucoLandingNode : public rclcpp::Node {
public:
  ArucoLandingNode() : Node("aruco_landing_node"), clock_{RCL_STEADY_TIME} {
    /* 통신 설정 */
    rclcpp::QoS pose_qos(rclcpp::KeepLast(10)); pose_qos.reliable();
    auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    sensor_qos.best_effort();

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/x500/target_pose", pose_qos,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr m){ pose_=*m; has_pose_=true; last_pose_=clock_.now(); });

    heading_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", sensor_qos,
        [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr m){ yaw_=m->heading; });

    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/command/twist",10);
    cmd_pub_   = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command",10);
    state_pub_ = create_publisher<std_msgs::msg::String>("~/state",10);

    timer_ = create_wall_timer(50ms, std::bind(&ArucoLandingNode::tick,this));
    RCLCPP_INFO(get_logger(),"ArucoLandingNode (no yaw control) ready");
  }
private:
  /************** FSM **************/
  enum class S{WAIT,COARSE,FINE,HOLD,SEARCH,DONE};
  S state_=S::WAIT, prev_state_=S::WAIT; rclcpp::Time state_enter_;
  void set_state(S s){ prev_state_=state_; state_=s; state_enter_=clock_.now(); std_msgs::msg::String m; m.data=state_names[(int)s]; state_pub_->publish(m);}  
  double since() { return (clock_.now()-state_enter_).seconds(); }
  const char* state_names[6]={"WAIT","COARSE","FINE","HOLD","SEARCH","DONE"};

  /************** ROS **************/
  rclcpp::Clock clock_; rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr heading_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  /************** State vars **************/
  bool has_pose_{false}; geometry_msgs::msg::PoseStamped pose_; rclcpp::Time last_pose_{clock_.now()};
  double yaw_{0.0};                     // 현재 헤딩(rad)
  geometry_msgs::msg::Twist last_cmd_;
  double last_alt_{10.0};              // 초기 고도 10 m
  double prev_ex_{0.0}, prev_ey_{0.0}; // 이전 오차 저장
  rclcpp::Time prev_t_{clock_.now()};  // 이전 오차 시각
  double coarse_stable_{0.0}, fine_stable_{0.0};

  /************** Main loop **************/
  void tick(){
    double dt_pose=(clock_.now()-last_pose_).seconds(); bool ok=has_pose_ && dt_pose<LOST_SHORT;
    if(state_==S::WAIT && !ok){ publish(0,0,VZ_SLOW); return; }

    /* pos error in body FLU -> FRD(right) -> NED */
    double ex=0, ey=0; double alt=last_alt_;
    if(ok){
      tf2::Vector3 p_rf(pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z);
      tf2::Quaternion q; tf2::fromMsg(pose_.pose.orientation,q);
      tf2::Vector3 err = -tf2::quatRotate(q.inverse(),p_rf);  // to marker
      ex = err.x();             // +forward
      ey = -err.y();            // FLU +left -> FRD +right (sign flip)
      alt = p_rf.z();
      last_alt_ = alt;           // +up RFU
    }

    /* forced land */
        // 강제 착륙 조건: "이미 한번이라도 인식한 후" + 저고도 + 장기 끊김
    if(has_pose_ && !ok && last_alt_ < CLOSE_ALT && dt_pose > LOST_LONG) { finish("force land: lost pose at low alt"); return; }

    /* FSM */
    switch(state_){
      case S::WAIT: if(ok) set_state(S::COARSE); break;
      case S::COARSE: {
        double vx,vy; pid(ex,ey,vx,vy,true); publish(vx,vy,VZ_SLOW);
        if(ok && hypot(ex,ey) < COARSE_THRESH) coarse_stable_ += 0.05; else coarse_stable_=0.0;
        if(coarse_stable_ > 0.3) set_state(S::FINE);
      } break;
      case S::FINE: {
        double vx,vy; pid(ex,ey,vx,vy,false); publish(vx,vy,VZ_FAST);
        if(ok && hypot(ex,ey) < FINE_THRESH && alt < 0.02) fine_stable_ += 0.05; else fine_stable_ = 0.0;
        if(fine_stable_ > 0.5) finish("landing complete");
      } break;
      case S::HOLD: {
        publish(0,0,0);
        if(ok) set_state(prev_state_);
        else if(since()>HOLD_TIMEOUT) set_state(S::SEARCH);
      } break;
      case S::SEARCH: {
        publish(0,0,SEARCH_UP_VEL);
        if(ok) set_state(prev_state_);
        else if(since()>SEARCH_TIMEOUT) finish("abort: marker lost");
      } break;
      case S::DONE: return;
    }

    if(!ok && state_!=S::HOLD && state_!=S::SEARCH && alt>CLOSE_ALT && dt_pose>LOST_SHORT)
      set_state(S::HOLD);

    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
      "[%s] err=%.2f alt=%.2f | v=[%.2f,%.2f,%.2f]",state_names[(int)state_],hypot(ex,ey),alt,
      last_cmd_.linear.x,last_cmd_.linear.y,last_cmd_.linear.z);
  }

  /************** helpers **************/
  void pid(double ex,double ey,double &vx,double &vy,bool coarse){
    double kp=coarse?KP_XY*0.7:KP_XY, kd=coarse?KD_XY*0.7:KD_XY;
    double dt=std::max((clock_.now()-prev_t_).seconds(),1e-3);
    double dex=(ex-prev_ex_)/dt, dey=(ey-prev_ey_)/dt; prev_ex_=ex; prev_ey_=ey; prev_t_=clock_.now();
    double vx_b=kp*ex+kd*dex, vy_b=kp*ey+kd*dey;                      // FRD body
    vx_b=std::clamp(vx_b,-MAX_XY_VEL,MAX_XY_VEL); vy_b=std::clamp(vy_b,-MAX_XY_VEL,MAX_XY_VEL);
    vx =  vx_b*std::cos(yaw_) - vy_b*std::sin(yaw_);
    vy =  vx_b*std::sin(yaw_) + vy_b*std::cos(yaw_);
  }
  void publish(double vx,double vy,double vz){ last_cmd_.linear.x=vx; last_cmd_.linear.y=vy; last_cmd_.linear.z=vz; last_cmd_.angular.z=0; twist_pub_->publish(last_cmd_); }
  void finish(const char* msg){ send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,0); RCLCPP_INFO(get_logger(),"%s",msg); state_=S::DONE; rclcpp::shutdown(); }
  void send_cmd(uint16_t cmd,float p1){ px4_msgs::msg::VehicleCommand m{}; m.timestamp=clock_.now().nanoseconds()/1000; m.command=cmd; m.param1=p1; m.target_system=1; m.target_component=1; m.source_system=1; m.source_component=1; m.from_external=true; cmd_pub_->publish(m);}  
};

int main(int argc,char** argv){ rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<ArucoLandingNode>()); rclcpp::shutdown(); }
