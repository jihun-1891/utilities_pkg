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
        send_gimbal_command(0.0);  // 정면
        break;
      case '1':  // 숫자패드 1
        send_gimbal_command(-90.0);  // 아래로
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

  void send_gimbal_command(float pitch_deg)
  {
    // 1. Configure gimbal mode (204)
    px4_msgs::msg::VehicleCommand cfg_cmd;
    cfg_cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    cfg_cmd.param1 = 1.0;  // stabilize pitch
    cfg_cmd.param2 = 0.0;  // stabilize roll
    cfg_cmd.param3 = 0.0;  // stabilize yaw
    cfg_cmd.param4 = NAN;
    cfg_cmd.param5 = NAN;
    cfg_cmd.param6 = NAN;
    cfg_cmd.param7 = 2.0;  // MAV_MOUNT_MODE_MAVLINK_TARGETING
    cfg_cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_MOUNT_CONFIGURE;  // 204
    cfg_cmd.target_system = 1;
    cfg_cmd.target_component = 1;
    cfg_cmd.source_system = 1;
    cfg_cmd.source_component = 1;
    cfg_cmd.from_external = true;
    vehicle_cmd_pub_->publish(cfg_cmd);

    rclcpp::sleep_for(std::chrono::milliseconds(50));  // PX4가 처리할 시간 살짝 주기

    // 2. Send pitch control command (205)
    px4_msgs::msg::VehicleCommand ctrl_cmd;
    ctrl_cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    ctrl_cmd.param1 = pitch_deg;   // pitch (deg)
    ctrl_cmd.param2 = 0.0f;        // roll (deg)
    ctrl_cmd.param3 = 0.0f;        // yaw (deg)
    ctrl_cmd.param4 = 0.0f;        // unused
    ctrl_cmd.param5 = 0.0f;
    ctrl_cmd.param6 = 0.0f;
    ctrl_cmd.param7 = NAN;
    ctrl_cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_MOUNT_CONTROL;  // 205
    ctrl_cmd.target_system = 1;
    ctrl_cmd.target_component = 1;
    ctrl_cmd.source_system = 1;
    ctrl_cmd.source_component = 1;
    ctrl_cmd.from_external = true;
    vehicle_cmd_pub_->publish(ctrl_cmd);
    RCLCPP_INFO(this->get_logger(), "Sent gimbal pitch command: %.1f degrees", pitch_deg);
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
