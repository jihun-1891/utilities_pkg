// Pose‑based autonomous landing node (no TF publishing)
// ------------------------------------------------------
//   * Receives marker pose on /x500/target_pose  (map frame)
//   * Waits for "/uav/cmd" == "rendezvous" then runs FSM
//   * Commands /command/pose with target pose in map frame
//   * State published on ~/state (std_msgs/String)

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>

using namespace std::chrono_literals;
/**************** 튜닝 상수 (ENU map) ****************/
constexpr double COARSE_THRESH    = 0.20;  // coarse XY 오차 [m]
constexpr double COARSE_WINDOW    = 0.30;  // 연속 유지 누적 [s]
constexpr double FINE_THRESH      = 0.07;  // fine  XY 오차 [m]
constexpr double FINE_WINDOW      = 0.40;  // 연속 유지 누적 [s]
constexpr double STABLE_DELAY     = 0.80;  // 단계 전환 지연 [s]
constexpr double LOST_SHORT       = 2.00;  // pose 단기 손실 허용 [s]
constexpr double HOLD_TIMEOUT     = 2.00;  // HOLD → SEARCH 전환 [s]
constexpr double SEARCH_UP_DIST   = 0.30;  // hold 중 상승량  [m]
constexpr double DESCENT_STEP_Z   = 0.45;  // fine 단계 강하량 [m]
constexpr double LAND_ALT_THRESH  = 0.03;  // 지상 판정  [m]
/****************************************************/

class PoseLandingNode : public rclcpp::Node
{
public:
  PoseLandingNode() : Node("pose_landing_node")
  {
    base_link_frame_ = declare_parameter("base_link_frame","x500_gimbal_0/base_link");

    clock_       = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf_buf_      = std::make_shared<tf2_ros::Buffer>(clock_);
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buf_);

    /* Subscribers */
    auto be_qos = rclcpp::QoS(10).best_effort();
    marker_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/x500/target_pose", be_qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {
        marker_pose_ = *msg;
        has_pose_    = true;
        last_pose_   = clock_->now();
      });

    cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "/uav/cmd", 10,
      [this](const std_msgs::msg::String::SharedPtr msg)
      {
        if(msg->data == "rendezvous")
        {
          start_ = true;
          RCLCPP_INFO(this->get_logger(), "Landing start triggered");
        }
      });

    /* Publishers */
    target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/command/pose",10);
    state_pub_  = create_publisher<std_msgs::msg::String>("~/state",10);

    /* Main loop */
    timer_ = create_wall_timer(50ms, std::bind(&PoseLandingNode::tick,this));

    RCLCPP_INFO(get_logger(), "PoseLandingNode ready – waiting for /uav/cmd='rendezvous'");
  }

private:
  /********* Finite‑state machine *********/
  enum class S{IDLE,WAIT,COARSE,STABLE_C,FINE,STABLE_F,HOLD,SEARCH,DONE};
  S st_   = S::IDLE;
  S prev_ = S::IDLE;
  rclcpp::Time st_enter_;
  void set_state(S s)
  {
    prev_ = st_;
    st_   = s;
    st_enter_ = clock_->now();
    std_msgs::msg::String m; m.data = state_name(st_);
    state_pub_->publish(m);
  }
  double since() const { return (clock_->now() - st_enter_).seconds(); }
  static const char* state_name(S s)
  {
    switch(s){
      case S::IDLE:     return "IDLE";
      case S::WAIT:     return "WAIT";
      case S::COARSE:   return "COARSE";
      case S::STABLE_C: return "STABLE_C";
      case S::FINE:     return "FINE";
      case S::STABLE_F: return "STABLE_F";
      case S::HOLD:     return "HOLD";
      case S::SEARCH:   return "SEARCH";
      case S::DONE:     return "DONE";
    }
    return "?";
  }

  /*********  ROS handles *********/
  std::string base_link_frame_;
  std::shared_ptr<rclcpp::Clock> clock_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr           cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr              state_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buf_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  /*********  State variables *********/
  geometry_msgs::msg::PoseStamped marker_pose_;
  bool has_pose_ = false;
  rclcpp::Time last_pose_;
  geometry_msgs::msg::Pose drone_pose_{};  // current drone pose (map)
  geometry_msgs::msg::Pose tgt_{};         // pose command (map)
  bool start_ = false;
  double c_acc_ = 0.0, f_acc_ = 0.0;

  /*********  Main loop *********/
  void tick()
  {
    update_drone_pose();

    if(st_ == S::IDLE)
    {
      if(start_) set_state(S::WAIT); else return;
    }

    publish_target();

    /* pose validity */
    bool   pose_ok = has_pose_ && (clock_->now() - last_pose_).seconds() < LOST_SHORT;
    double alt     = drone_pose_.position.z;
    double dx = 0.0, dy = 0.0, err = 0.0;
    if(pose_ok)
    {
      dx  = marker_pose_.pose.position.x - drone_pose_.position.x;
      dy  = marker_pose_.pose.position.y - drone_pose_.position.y;
      err = std::hypot(dx,dy);
    }

    /* FSM step */
    switch(st_)
    {
      case S::WAIT:      if(pose_ok)                  set_state(S::COARSE);                    break;
      case S::COARSE:    coarse(pose_ok,dx,dy,err);                                           break;
      case S::STABLE_C:  if(since()>STABLE_DELAY)     set_state(S::FINE);                      break;
      case S::FINE:      fine  (pose_ok,dx,dy,err);                                           break;
      case S::STABLE_F:  if(since()>STABLE_DELAY)     set_state(S::DONE);                      break;
      case S::HOLD:      hold  (pose_ok);                                                      break;
      case S::SEARCH:    search(pose_ok);                                                     break;
      case S::DONE:      return;                                                               
      default: break;
    }

    /* lost pose -> HOLD */
    if(!pose_ok && st_!=S::HOLD && st_!=S::SEARCH && alt>0.2)
      set_state(S::HOLD);

    RCLCPP_INFO_THROTTLE(get_logger(), *clock_, 1000,
      "[%s] ok=%d err=%.2f alt=%.2f tgtZ=%.2f", state_name(st_), pose_ok, err, alt, tgt_.position.z);
  }

  /*********  State helpers *********/
  void coarse(bool ok,double dx,double dy,double err)
  {
    tgt_.position.x = drone_pose_.position.x + dx;
    tgt_.position.y = drone_pose_.position.y + dy;
    tgt_.position.z = drone_pose_.position.z;                // keep Z

    if(ok && err < COARSE_THRESH) c_acc_ += 0.05; else c_acc_ = 0.0;
    if(c_acc_ > COARSE_WINDOW) { c_acc_ = 0.0; set_state(S::STABLE_C); }
  }

  void fine(bool ok,double dx,double dy,double err)
  {
    tgt_.position.x = drone_pose_.position.x + dx;
    tgt_.position.y = drone_pose_.position.y + dy;
    tgt_.position.z = drone_pose_.position.z - DESCENT_STEP_Z;

    if(ok && err < FINE_THRESH) f_acc_ += 0.05; else f_acc_ = 0.0;
    if(f_acc_ > FINE_WINDOW && tgt_.position.z < LAND_ALT_THRESH)
    {
      f_acc_ = 0.0;
      set_state(S::STABLE_F);
    }
  }

  void hold(bool ok)
  {
    tgt_.position = drone_pose_.position;            // hover
    if(ok)
      set_state(prev_);
    else if(since() > HOLD_TIMEOUT)
    {
      tgt_.position.z += SEARCH_UP_DIST;             // rise a bit then search
      set_state(S::SEARCH);
    }
  }

  void search(bool ok)
  {
    if(ok)
      set_state(prev_);            // regained pose -> back to previous
    else if(since() > HOLD_TIMEOUT)
      set_state(S::DONE);          // give up
  }

  /* ------ pose helpers ------ */
  void update_drone_pose()
  {
    try
    {
      auto t = tf_buf_->lookupTransform("map", base_link_frame_, tf2::TimePointZero);
      drone_pose_.position.x = t.transform.translation.x;
      drone_pose_.position.y = t.transform.translation.y;
      drone_pose_.position.z = t.transform.translation.z;
      drone_pose_.orientation = t.transform.rotation;
    }
    catch(const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 2000, "tf lookup fail: %s", ex.what());
    }
  }

  void publish_target()
  {
    geometry_msgs::msg::PoseStamped m;
    m.header.stamp = clock_->now();
    m.header.frame_id = "map";
    m.pose = tgt_;
    target_pub_->publish(m);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseLandingNode>());
  rclcpp::shutdown();
  return 0;
}
