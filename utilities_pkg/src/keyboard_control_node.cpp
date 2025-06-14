#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>

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
    
    std::cout << "Keyboard control started.\n";
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr heading_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  double heading_rad_ = 0.0;  // current yaw in radians (from NED north)
  double vx_body_ = 0.0;
  double vy_body_ = 0.0;
  double vz_ = 0.0;
  double yaw_rate_ = 0.0;
  
  const double STEP = 0.5;
  const double YAW_SETP = 0.1;
  const double MAX_SPEED = 3.0;

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
      case 's':
        vx_body_ = vy_body_ = vz_ = yaw_rate_ = 0.0;
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
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardControl>());
  rclcpp::shutdown();
  std::cout << "\n종료됨\n";
  return 0;
}
