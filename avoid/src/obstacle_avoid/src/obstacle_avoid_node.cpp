#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <limits>
#include <string>
#include <cmath>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>

class ObstacleAvoidNode {
public:
  ObstacleAvoidNode() : nh_(), pnh_("~") {
    pnh_.param<std::string>("scan_topic", scan_topic_, std::string("/scan"));
    pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
    pnh_.param<double>("safe_dist", safe_dist_, 0.6);
    pnh_.param<double>("forward_speed", forward_speed_, 0.2);
    pnh_.param<double>("turn_speed", turn_speed_, 0.6);
    pnh_.param<bool>("allow_no_scan", allow_no_scan_, true);           // 允许无雷达数据时也发布指令（方便触碰调试）
    pnh_.param<double>("no_scan_timeout", no_scan_timeout_, 1.0);      // 超过该时间未收到scan则走fallback
    pnh_.param<double>("no_scan_linear", no_scan_linear_, 0.1);        // 无scan时的线速度
    pnh_.param<double>("no_scan_angular", no_scan_angular_, 0.0);      // 无scan时的角速度
  pnh_.param<bool>("log_enable", log_enable_, true);
  pnh_.param<std::string>("log_dir", log_dir_, std::string("/home/bcsh/workspace/ex4pcyy/avoid/logs"));
  openLog();
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
          << " front_angle_deg=" << front_angle_deg
          << " allow_no_scan=" << std::boolalpha << allow_no_scan_
          << " no_scan_timeout=" << no_scan_timeout_
          << " no_scan_linear=" << no_scan_linear_
          << " no_scan_angular=" << no_scan_angular_
          << " log_enable=" << log_enable_);
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
    last_scan_time_ = ros::Time::now();
    log("scan", "rx", last_scan_time_.toSec());
  }

  void publishCmd() {
    geometry_msgs::Twist cmd;
    const ros::Time now = ros::Time::now();

    if (!has_scan_) {
      if (allow_no_scan_) {
        cmd.linear.x = no_scan_linear_;
        cmd.angular.z = no_scan_angular_;
        cmd_pub_.publish(cmd);
        logDecision("fallback_no_scan", cmd.linear.x, cmd.angular.z, std::numeric_limits<double>::quiet_NaN());
      }
      return;
    }

    if (allow_no_scan_ && no_scan_timeout_ > 0.0 && !last_scan_time_.isZero()) {
      double dt = (now - last_scan_time_).toSec();
      if (dt > no_scan_timeout_) {
        cmd.linear.x = no_scan_linear_;
        cmd.angular.z = no_scan_angular_;
        cmd_pub_.publish(cmd);
        logDecision("fallback_timeout", cmd.linear.x, cmd.angular.z, std::numeric_limits<double>::quiet_NaN());
        return;
      }
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
      logDecision("turn", cmd.linear.x, cmd.angular.z, min_front);
    } else {
      // clear: go forward
      cmd.linear.x = forward_speed_;
      cmd.angular.z = 0.0;
      logDecision("forward", cmd.linear.x, cmd.angular.z, min_front);
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
  bool allow_no_scan_{true};
  double no_scan_timeout_{1.0};
  double no_scan_linear_{0.1};
  double no_scan_angular_{0.0};
  bool log_enable_{true};
  std::string log_dir_;
  std::ofstream log_file_;

  ros::Time last_scan_time_;

  sensor_msgs::LaserScan last_scan_;
  bool has_scan_{false};

  std::string nowString() {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto tt = system_clock::to_time_t(now);
    auto tm = *std::localtime(&tt);
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    std::ostringstream oss;
    oss << std::put_time(&tm, "%F %T") << '.' << std::setw(3) << std::setfill('0') << ms.count();
    return oss.str();
  }

  void openLog() {
    if (!log_enable_) return;
    std::string cmd = "mkdir -p '" + log_dir_ + "'";
    std::system(cmd.c_str());
    std::time_t tt = std::time(nullptr);
    std::tm tm = *std::localtime(&tt);
    std::ostringstream oss;
    oss << log_dir_ << "/obstacle_avoid_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".log";
    log_file_.open(oss.str(), std::ios::out | std::ios::trunc);
    if (log_file_.is_open()) {
      log_file_ << nowString() << " log_start" << std::endl;
    }
  }

  void log(const std::string &tag, const std::string &msg, double value) {
    if (!log_enable_ || !log_file_.is_open()) return;
    log_file_ << nowString() << " [" << tag << "] " << msg << " " << value << std::endl;
  }

  void logDecision(const std::string &mode, double lin, double ang, double min_front) {
    if (!log_enable_ || !log_file_.is_open()) return;
    log_file_ << nowString() << " [decide] mode=" << mode
              << " lin=" << lin << " ang=" << ang;
    if (!std::isnan(min_front)) log_file_ << " min_front=" << min_front;
    log_file_ << std::endl;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "obstacle_avoid_node");
  ObstacleAvoidNode node;
  node.spin();
  return 0;
}
