#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <map>

char getch() {
  struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  char c = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return c;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("keyboard_velocity_teleop");
  auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/command/twist", 10);

  double speed = 1.0;
  RCLCPP_INFO(node->get_logger(), "Controls: WASD + J/K/L for up/down/stop, Q/E to rotate");

  std::map<char, std::function<geometry_msgs::msg::Twist()>> keymap = {
    {'w', [speed](){ geometry_msgs::msg::Twist t; t.linear.x = speed; return t; }},
    {'s', [speed](){ geometry_msgs::msg::Twist t; t.linear.x = -speed; return t; }},
    {'a', [speed](){ geometry_msgs::msg::Twist t; t.linear.y = speed; return t; }},
    {'d', [speed](){ geometry_msgs::msg::Twist t; t.linear.y = -speed; return t; }},
    {'j', [speed](){ geometry_msgs::msg::Twist t; t.linear.z = speed; return t; }},
    {'k', [speed](){ geometry_msgs::msg::Twist t; t.linear.z = -speed; return t; }},
    {'q', [speed](){ geometry_msgs::msg::Twist t; t.angular.z = speed; return t; }},
    {'e', [speed](){ geometry_msgs::msg::Twist t; t.angular.z = -speed; return t; }},
    {'l', [](){ return geometry_msgs::msg::Twist(); }}
  };

  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok()) {
    char c = getch();
    if (keymap.count(c)) {
      pub->publish(keymap[c]());
    }
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
