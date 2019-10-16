#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
using std::placeholders::_1;

class MonitorObstacleDistance : public rclcpp::Node
{
public:
  MonitorObstacleDistance()
  : Node("monitor_obstacle_distance_cpp")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&MonitorObstacleDistance::report_metric, this, _1));
  }

private:
  std::vector<float> filter_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "Filtering scan values in value range (%d, %d)", msg->range_min, msg->range_max);
    std::vector<float> filtered_scan;
    for (int i = 0; i < 360; i++) {
      filtered_scan.push_back(msg->ranges[i]);
    }
    return filtered_scan;
  }

  void report_metric(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Reporting metric");
    std::vector<float> filtered_scan = this->filter_scan(msg);

    auto min_distance = *std::min_element(filtered_scan.begin(), filtered_scan.end());
    RCLCPP_INFO(this->get_logger(), "Nearest obstacle: %d", min_distance);
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonitorObstacleDistance>());
  rclcpp::shutdown();
  return 0;
}
