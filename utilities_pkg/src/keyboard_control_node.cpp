#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>

// For Pose TF
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <fstream>

using VC = px4_msgs::msg::VehicleCommand;

class KeyboardControl : public rclcpp::Node
{
public:
  KeyboardControl() : Node("keyboard_control_node")
  {
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/command/twist", 10);
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    heading_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position",
      qos,
      std::bind(&KeyboardControl::heading_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&KeyboardControl::timer_callback, this));
    
    vehicle_cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

    std::cout << "Keyboard control started.\n";

    uav_file_.open("uav_wp.csv", std::ios::app);
    ugv_file_.open("ugv_wp.csv", std::ios::app);
    if (uav_file_.tellp() == 0) uav_file_ << "x,y,z\n";
    if (ugv_file_.tellp() == 0) ugv_file_ << "x,y,z\n";

    RCLCPP_INFO(get_logger(), "Keyboard control started.");
    
    // gimbal_cfg_timer_ = this->create_wall_timer(
    //   std::chrono::milliseconds(1000),   /* 1 초 후 한 번만 실행 */
    //   [this](){
    //     if (!control_taken_)  take_gimbal_control();
    //     gimbal_cfg_timer_->cancel();   // 더 돌 필요 없음
    //   });
    take_gimbal_control();
    RCLCPP_INFO(get_logger(), "Request gimbal order of priority.");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr heading_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  double heading_rad_ = 0.0;  // current yaw in radians (from NED north)
  double vx_body_ = 0.0;
  double vy_body_ = 0.0;
  double vz_ = 0.0;
  double yaw_rate_ = 0.0;
  
  const double STEP = 0.5;
  const double YAW_SETP = 0.1;
  const double MAX_SPEED = 3.0;

  const uint8_t MY_SYSID  = 46;   // 비행 컨트롤러와 동일
  const uint8_t MY_COMPID = 47;  // USER1
  const uint8_t TARGET_SYSID  = 1;   
  const uint8_t TARGET_COMPID = 1;  


  bool control_taken_ = false;                      // 제어권 확보 여부
  rclcpp::TimerBase::SharedPtr gimbal_cfg_timer_;   // 1-회 타이머
  
  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::ofstream uav_file_;
  std::ofstream ugv_file_;

  void heading_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    heading_rad_ = msg->heading;  // Already in radians
  }

  void limit(double &v)
  {
    if (v > MAX_SPEED) v = MAX_SPEED;
    else if (v < -MAX_SPEED) v = -MAX_SPEED;
  }

  char getch()
  {
    char buf = 0;
    termios old = {};
    if (tcgetattr(STDIN_FILENO, &old) < 0) perror("tcgetattr()");
    termios new_term = old;
    new_term.c_lflag &= ~(ICANON | ECHO);
    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_term) < 0) perror("tcsetattr()");
    if (read(STDIN_FILENO, &buf, 1) < 0) perror("read()");
    if (tcsetattr(STDIN_FILENO, TCSADRAIN, &old) < 0) perror("tcsetattr()");
    return buf;
  }

  void timer_callback()
  {
    char c = getch();

    switch (c)
    {
      case 'w': vx_body_ += STEP; break;
      case 'x': vx_body_ -= STEP; break;
      case 'a': vy_body_ -= STEP; break;
      case 'd': vy_body_ += STEP; break;
      case 'q': vz_ += STEP; break;
      case 'e': vz_ -= STEP; break;
      case 'z': yaw_rate_ -= YAW_SETP; break;
      case 'c': yaw_rate_ += YAW_SETP; break;
      case '0':  // 숫자패드 0
        send_gimbal_pitch(0.0);  // 정면
        break;
      case '1':  // 숫자패드 1
        send_gimbal_pitch(-90.0);  // 아래로
        break;
      case 's':
        vx_body_ = vy_body_ = vz_ = yaw_rate_ = 0.0;
        break;
      case 'o':   // UAV waypoint 저장
        save_waypoint("x500_gimbal_0/base_link", uav_file_);
        break;
      case 'p':   // UGV waypoint 저장
        save_waypoint("X1_asp/base_link", ugv_file_);
        break;
      default:
        return;
    }

    limit(vx_body_);
    limit(vy_body_);
    limit(vz_);
    limit(yaw_rate_);

    // Body → NED 변환
    geometry_msgs::msg::Twist twist;
    twist.linear.x = vx_body_ * cos(heading_rad_) - vy_body_ * sin(heading_rad_);
    twist.linear.y = vx_body_ * sin(heading_rad_) + vy_body_ * cos(heading_rad_);
    twist.linear.z = vz_;
    twist.angular.z = yaw_rate_;

    twist_pub_->publish(twist);

    std::cout << std::fixed << std::setprecision(2)
              << "\rvx_body: " << vx_body_ << " vy_body: " << vy_body_
              << " -> NED x: " << twist.linear.x << " y: " << twist.linear.y
              << " | heading(deg): " << heading_rad_ * 180.0 / M_PI << "       " << std::flush;
  }

  void save_waypoint(const std::string &child_frame, std::ofstream &file)
  {
    try
    {
      geometry_msgs::msg::TransformStamped tf =
          tf_buffer_.lookupTransform("map", child_frame, tf2::TimePointZero);

      double x = tf.transform.translation.x;
      double y = tf.transform.translation.y;
      double z = tf.transform.translation.z;

      file << x << "," << y << "," << z << "\n";
      file.flush();

      RCLCPP_INFO(this->get_logger(),
                  "Saved WP for %s  (%.2f, %.2f, %.2f)",
                  child_frame.c_str(), x, y, z);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    }
  }

  void send_gimbal_pitch(float pitch_deg)
  {
    take_gimbal_control();
    /* 2) 피치 명령 */
    auto now_us = this->get_clock()->now().nanoseconds() / 1000;

    VC cmd{};
    cmd.timestamp          = now_us + 1000;
    cmd.command            = VC::VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW; // 1000
    cmd.param1             = pitch_deg;   // Pitch (+위, -아래)
    cmd.param2             = NAN;         // Yaw 유지
    cmd.param5             = 0;           // follow(0) or lock(16)
    cmd.param7             = 0;           // gimbal_device_id
    cmd.target_system      = TARGET_SYSID;
    cmd.target_component   = TARGET_COMPID;
    cmd.source_system      = MY_SYSID;
    cmd.source_component   = MY_COMPID;   // ★ 반드시 동일
    cmd.from_external      = true;
    vehicle_cmd_pub_->publish(cmd);
  }

  void take_gimbal_control()
  {
      VC cfg{};
      cfg.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
      cfg.command          = VC::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;   // 1001

      /* ── 핵심: -2 를 넣으면 PX4가 msg 의 sys/comp id 그대로 채움 ── */
      cfg.param1 = MY_SYSID;          // primary sysid  = msg->sysid  (1)
      cfg.param2 = MY_COMPID;          // primary compid = msg->compid (51)
      cfg.param3 = cfg.param4 = 0;     // secondary 없음
      cfg.param5 = 0;                   // flags
      cfg.param7 = 0;                   // gimbal device id

      cfg.target_system      = TARGET_SYSID;       // PX4
      cfg.target_component   = TARGET_COMPID;       // MAV_COMP_ID_AUTOPILOT
      cfg.source_system      = MY_SYSID;
      cfg.source_component   = MY_COMPID;
      cfg.from_external      = true;

      vehicle_cmd_pub_->publish(cfg);
      RCLCPP_INFO(get_logger(), ">>> Sent CONFIGURE(1001) – take primary control");
      control_taken_ = true;
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardControl>());
  rclcpp::shutdown();
  std::cout << "\n종료됨\n";
  return 0;
}
