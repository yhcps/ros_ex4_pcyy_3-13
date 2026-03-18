#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <limits>
#include <string>
#include <cmath>

class ObstacleAvoidNode {
public:
  ObstacleAvoidNode() : nh_(), pnh_("~") {
    pnh_.param<std::string>("scan_topic", scan_topic_, std::string("/scan"));
    pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
    pnh_.param<double>("safe_dist", safe_dist_, 0.6);
    pnh_.param<double>("forward_speed", forward_speed_, 0.2);
    pnh_.param<double>("turn_speed", turn_speed_, 0.6);
    double front_angle_deg;
    pnh_.param<double>("front_angle_deg", front_angle_deg, 60.0);
    front_angle_rad_ = front_angle_deg * M_PI / 180.0;

    scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(scan_topic_, 10, &ObstacleAvoidNode::scanCb, this);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);

    ROS_INFO_STREAM("obstacle_avoid_node started. scan_topic=" << scan_topic_
                    << " cmd_vel_topic=" << cmd_vel_topic_
                    << " safe_dist=" << safe_dist_
                    << " forward_speed=" << forward_speed_
                    << " turn_speed=" << turn_speed_
                    << " front_angle_deg=" << front_angle_deg);
  }

  void spin() {
    ros::Rate rate(15.0);
    while (ros::ok()) {
      ros::spinOnce();
      publishCmd();
      rate.sleep();
    }
  }

private:
  void scanCb(const sensor_msgs::LaserScan::ConstPtr &msg) {
    last_scan_ = *msg;
    has_scan_ = true;
  }

  void publishCmd() {
    geometry_msgs::Twist cmd;
    if (!has_scan_) {
      cmd_pub_.publish(cmd);
      return;
    }

    double min_front = std::numeric_limits<double>::infinity();
    const auto &msg = last_scan_;
    for (size_t i = 0; i < msg.ranges.size(); ++i) {
      double angle = msg.angle_min + static_cast<double>(i) * msg.angle_increment;
      if (std::fabs(angle) > front_angle_rad_ * 0.5) continue;
      double r = msg.ranges[i];
      if (std::isnan(r) || std::isinf(r)) continue;
      if (r < msg.range_min || r > msg.range_max) continue;
      if (r < min_front) min_front = r;
    }

    if (min_front < safe_dist_) {
      // obstacle too close: turn in place
      cmd.angular.z = turn_speed_;
      cmd.linear.x = 0.0;
    } else {
      // clear: go forward
      cmd.linear.x = forward_speed_;
      cmd.angular.z = 0.0;
    }

    cmd_pub_.publish(cmd);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber scan_sub_;
  ros::Publisher cmd_pub_;

  std::string scan_topic_;
  std::string cmd_vel_topic_;
  double safe_dist_{};
  double forward_speed_{};
  double turn_speed_{};
  double front_angle_rad_{};

  sensor_msgs::LaserScan last_scan_;
  bool has_scan_{false};
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "obstacle_avoid_node");
  ObstacleAvoidNode node;
  node.spin();
  return 0;
}
