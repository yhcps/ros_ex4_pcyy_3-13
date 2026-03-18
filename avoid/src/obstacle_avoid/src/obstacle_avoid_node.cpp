#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
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
    pnh_.param<std::string>("bump_topic", bump_topic_, std::string("/robot/bump_sensor"));
    pnh_.param<bool>("enable_bump", enable_bump_, true);
    pnh_.param<double>("bump_timeout", bump_timeout_, 0.6);            // bump 触发维持时间
    pnh_.param<double>("bump_back_speed", bump_back_speed_, -0.08);    // bump 时的后退速度
    pnh_.param<double>("bump_turn_speed", bump_turn_speed_, 0.5);      // bump 时的转向角速度基准
  pnh_.param<bool>("log_enable", log_enable_, true);
  pnh_.param<std::string>("log_dir", log_dir_, std::string("/home/bcsh/workspace/ex4pcyy/avoid/logs"));
  openLog();
    double front_angle_deg;
    pnh_.param<double>("front_angle_deg", front_angle_deg, 60.0);
    front_angle_rad_ = front_angle_deg * M_PI / 180.0;

    scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(scan_topic_, 10, &ObstacleAvoidNode::scanCb, this);
    if (enable_bump_) {
      bump_sub_ = nh_.subscribe<std_msgs::Int32MultiArray>(bump_topic_, 10, &ObstacleAvoidNode::bumpCb, this);
    }
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);

  ROS_INFO_STREAM("obstacle_avoid_node started. scan_topic=" << scan_topic_
          << " cmd_vel_topic=" << cmd_vel_topic_
          << " safe_dist=" << safe_dist_
          << " forward_speed=" << forward_speed_
          << " turn_speed=" << turn_speed_
          << " front_angle_deg=" << front_angle_deg
    << " enable_bump=" << enable_bump_
    << " bump_topic=" << bump_topic_
    << " bump_timeout=" << bump_timeout_
    << " bump_back_speed=" << bump_back_speed_
    << " bump_turn_speed=" << bump_turn_speed_
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

  void bumpCb(const std_msgs::Int32MultiArray::ConstPtr &msg) {
    last_bump_ = *msg;
    last_bump_time_ = ros::Time::now();
    has_bump_ = true;
    int hits = 0;
    for (auto v : msg->data) hits += (v != 0);
    log("bump", "rx_hits", static_cast<double>(hits));
  }

  void publishCmd() {
    geometry_msgs::Twist cmd;
    const ros::Time now = ros::Time::now();

    if (enable_bump_ && has_bump_ && !last_bump_time_.isZero()) {
      double dt_bump = (now - last_bump_time_).toSec();
      if (dt_bump <= bump_timeout_ && isBumpHit()) {
        calcBumpCmd(cmd);
        cmd_pub_.publish(cmd);
        logDecision("bump", cmd.linear.x, cmd.angular.z, std::numeric_limits<double>::quiet_NaN());
        return; // bump 优先级最高，直接返回
      }
    }

    if (!has_scan_) {
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
  ros::Subscriber bump_sub_;
  ros::Publisher cmd_pub_;

  std::string scan_topic_;
  std::string cmd_vel_topic_;
  std::string bump_topic_;
  double safe_dist_{};
  double forward_speed_{};
  double turn_speed_{};
  double front_angle_rad_{};
  bool enable_bump_{true};
  double bump_timeout_{0.6};
  double bump_back_speed_{-0.08};
  double bump_turn_speed_{0.5};
  bool log_enable_{true};
  std::string log_dir_;
  std::ofstream log_file_;

  ros::Time last_scan_time_;
  ros::Time last_bump_time_;

  sensor_msgs::LaserScan last_scan_;
  bool has_scan_{false};
  std_msgs::Int32MultiArray last_bump_;
  bool has_bump_{false};

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

  bool isBumpHit() const {
    if (!has_bump_) return false;
    for (auto v : last_bump_.data) {
      if (v != 0) return true;
    }
    return false;
  }

  void calcBumpCmd(geometry_msgs::Twist &cmd) const {
    // 0/1/2 位：左前 / 中前 / 右前，>0 视为触发
    bool left = last_bump_.data.size() > 0 && last_bump_.data[0] != 0;
    bool center = last_bump_.data.size() > 1 && last_bump_.data[1] != 0;
    bool right = last_bump_.data.size() > 2 && last_bump_.data[2] != 0;

    cmd.linear.x = bump_back_speed_;
    cmd.angular.z = 0.0;

    if (left && !right) {
      // 左触发，右转（负角速度）
      cmd.angular.z = -std::fabs(bump_turn_speed_);
    } else if (right && !left) {
      // 右触发，左转（正角速度）
      cmd.angular.z = std::fabs(bump_turn_speed_);
    } else if (center && !left && !right) {
      // 只中间，后退不转
      cmd.angular.z = 0.0;
    } else if (left && right) {
      // 左右同时，后退并略微左转帮助脱离
      cmd.angular.z = std::fabs(bump_turn_speed_) * 0.5;
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "obstacle_avoid_node");
  ObstacleAvoidNode node;
  node.spin();
  return 0;
}
