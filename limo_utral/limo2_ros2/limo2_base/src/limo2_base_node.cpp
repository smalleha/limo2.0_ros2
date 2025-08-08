
#include "limo2_base/limo2_driver.h"
using namespace AgileX;
std::shared_ptr<Limo2Driver> robot;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Limo2Driver>("limo_base"));
  rclcpp::Rate rate(20);
  rate.sleep();
  rclcpp::shutdown();

  return 0;
}
