// imu_turn180.cpp
// ROS1 node: use IMU yaw (with optional zero-offset) to rotate in place by a target angle (default 180°)
// Reference: ex2pcyy ros_walk1x1.cpp

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <limits>
#include <fstream>
#include <sstream>
#include <chrono>
#include <ctime>
#include <iomanip>

struct ImuState {
  double yaw_raw{0.0}; // rad
  bool has_imu{false};
  bool offset_enable{true};
  double initial_param{std::numeric_limits<double>::quiet_NaN()};
  double initial_measured{std::numeric_limits<double>::quiet_NaN()};
  double offset{0.0};
  bool offset_ready{false};
} g_imu;

static std::ofstream g_log;
static std::string g_log_path;

std::string now_string() {
  using namespace std::chrono;
  auto now = system_clock::now();
  auto tt = system_clock::to_time_t(now);
  auto tm = *std::localtime(&tt);
  auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
  std::ostringstream oss;
  oss << std::put_time(&tm, "%F %T") << '.' << std::setw(3) << std::setfill('0') << ms.count();
  return oss.str();
}

void log_line(const std::string &msg) {
  if (g_log.is_open()) {
    g_log << now_string() << " " << msg << std::endl;
  }
}

// normalize angle to (-pi, pi]
double normalize_angle(double a) {
  while (a > M_PI) a -= 2 * M_PI;
  while (a <= -M_PI) a += 2 * M_PI;
  return a;
}

// shortest angular distance target-current in (-pi, pi]
double shortest_angular_distance(double current, double target) {
  return normalize_angle(target - current);
}

double yaw_from_quat(const geometry_msgs::Quaternion &q) {
  tf2::Quaternion tfq(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tfq).getRPY(roll, pitch, yaw);
  return yaw;
}

// IMU callback
void imu_cb(const sensor_msgs::Imu::ConstPtr &msg) {
  g_imu.yaw_raw = yaw_from_quat(msg->orientation);
  g_imu.has_imu = true;

  if (g_imu.offset_enable && !g_imu.offset_ready) {
    const double measured = g_imu.yaw_raw;
    if (std::isnan(g_imu.initial_param)) {
      g_imu.initial_measured = measured;
      g_imu.offset = normalize_angle(-measured); // zero at power-on heading
    } else {
      g_imu.initial_measured = g_imu.initial_param;
      g_imu.offset = normalize_angle(g_imu.initial_param - measured);
    }
    g_imu.offset_ready = true;
    ROS_INFO("IMU offset ready: measured=%.3f, initial=%.3f, offset=%.3f", measured, g_imu.initial_measured, g_imu.offset);
  }
}

double imu_yaw_corrected() {
  double yaw = g_imu.yaw_raw;
  if (g_imu.offset_ready) yaw = normalize_angle(yaw + g_imu.offset);
  return yaw;
}

// Closed-loop turn using IMU yaw
bool turn_angle_imu(ros::Publisher &pub, ros::Rate &rate,
                    double angle_rad, double angular_speed,
                    double min_angular_speed, double timeout_s,
                    double kp) {
  if (!g_imu.has_imu) {
    ROS_ERROR("turn_angle_imu: no IMU received yet");
    return false;
  }
  if (std::abs(angle_rad) < 1e-6 || angular_speed <= 0) {
    ROS_ERROR("turn_angle_imu: invalid angle or speed");
    return false;
  }

  const double start_yaw = imu_yaw_corrected();
  const double target_yaw = normalize_angle(start_yaw + angle_rad);
  double yaw_err = shortest_angular_distance(imu_yaw_corrected(), target_yaw);
  double sign = (yaw_err >= 0) ? 1.0 : -1.0;

  geometry_msgs::Twist cmd;
  const double w_min = std::max(0.05, min_angular_speed);
  const double w_max = angular_speed;
  const double tol = 1.0 * M_PI / 180.0; // 1 degree
  double last_err = yaw_err;
  ros::Time t0 = ros::Time::now();

  ROS_INFO("Turn start: start=%.3f rad, target=%.3f rad (err=%.3f)", start_yaw, target_yaw, yaw_err);

  while (ros::ok()) {
    yaw_err = shortest_angular_distance(imu_yaw_corrected(), target_yaw);
    double w_cmd = kp * yaw_err;
    if (std::abs(w_cmd) < w_min) w_cmd = sign * w_min;
    w_cmd = std::max(-w_max, std::min(w_max, w_cmd));
    cmd.angular.z = w_cmd;

    pub.publish(cmd);
    ros::spinOnce();
    rate.sleep();

    if (std::abs(yaw_err) < tol) break;

    if ((yaw_err > 0) != (last_err > 0) && std::abs(yaw_err) < 3 * tol) {
      // crossed target with small residual
      break;
    }
    last_err = yaw_err;

    if ((ros::Time::now() - t0).toSec() > timeout_s) {
      ROS_WARN("turn_angle_imu: timeout (err=%.3f rad)", yaw_err);
      break;
    }
  }

  // brake
  geometry_msgs::Twist zero;
  pub.publish(zero);
  ROS_INFO("Turn done: now=%.3f rad, err=%.3f rad", imu_yaw_corrected(), yaw_err);
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_turn180_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  double target_deg = 180.0;
  double angular_speed = 0.8;        // rad/s cap
  double min_angular_speed = 0.3;     // rad/s to overcome stiction
  double timeout_s = 12.0;            // seconds
  double kp = 1.6;                    // proportional gain (raised for stronger end control)
  double brake_time = 0.3;            // seconds pre/post brake
  std::string imu_topic = "/imu/data";
  std::string cmd_vel_topic = "/cmd_vel";
  bool imu_offset_enable = true;
  double imu_initial_yaw = std::numeric_limits<double>::quiet_NaN();

  pnh.param("target_deg", target_deg, target_deg);
  pnh.param("angular_speed", angular_speed, angular_speed);
  pnh.param("min_angular_speed", min_angular_speed, min_angular_speed);
  pnh.param("timeout_s", timeout_s, timeout_s);
  pnh.param("kp", kp, kp);
  pnh.param("brake_time", brake_time, brake_time);
  pnh.param("imu_topic", imu_topic, imu_topic);
  pnh.param("cmd_vel_topic", cmd_vel_topic, cmd_vel_topic);
  pnh.param("imu_offset_enable", imu_offset_enable, imu_offset_enable);
  pnh.param("imu_initial_yaw", imu_initial_yaw, imu_initial_yaw);

  g_imu.offset_enable = imu_offset_enable;
  g_imu.initial_param = imu_initial_yaw;

  // init log file under workspace/imu/logs
  {
    const std::string log_dir_default = "/home/bcsh/workspace/ex4pcyy/imu/logs";
    std::string log_dir = log_dir_default;
    std::system(("mkdir -p '" + log_dir + "'").c_str());
    std::time_t tt = std::time(nullptr);
    std::tm tm = *std::localtime(&tt);
    std::ostringstream oss;
    oss << log_dir << "/imu_turn180_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".log";
    g_log_path = oss.str();
    g_log.open(g_log_path.c_str(), std::ios::out | std::ios::trunc);
    if (g_log.is_open()) {
      log_line("log start");
    }
  }

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 50, imu_cb);
  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 10);
  ros::Rate rate(30.0);

  ROS_INFO("imu_turn180_node started. target=%.1f deg, w=%.2f rad/s, min_w=%.2f, kp=%.2f, imu=%s, cmd_vel=%s",
           target_deg, angular_speed, min_angular_speed, kp, imu_topic.c_str(), cmd_vel_topic.c_str());
  {
    std::ostringstream oss;
    oss << "params target_deg=" << target_deg
        << " w=" << angular_speed
        << " min_w=" << min_angular_speed
        << " kp=" << kp
        << " timeout=" << timeout_s
        << " brake_time=" << brake_time
        << " imu_topic=" << imu_topic
        << " cmd_vel_topic=" << cmd_vel_topic
        << " imu_offset_enable=" << imu_offset_enable
        << " imu_initial_yaw=" << imu_initial_yaw;
    log_line(oss.str());
  }

  // Wait for IMU
  ros::Time wait_imu = ros::Time::now();
  while (ros::ok() && !g_imu.has_imu) {
    ros::spinOnce();
    rate.sleep();
    if ((ros::Time::now() - wait_imu).toSec() > 5.0) {
      ROS_ERROR("No IMU messages within 5s on %s", imu_topic.c_str());
      log_line("no_imu_timeout topic=" + imu_topic);
      return 1;
    }
  }

  // Wait offset capture if enabled
  if (imu_offset_enable) {
    ros::Time wait_off = ros::Time::now();
    while (ros::ok() && !g_imu.offset_ready) {
      ros::spinOnce();
      rate.sleep();
      if ((ros::Time::now() - wait_off).toSec() > 5.0) {
        ROS_WARN("IMU offset not ready after 5s, continue with raw yaw");
        log_line("imu_offset_wait_timeout");
        break;
      }
    }
  }

  // Pre-brake
  cmd_pub.publish(geometry_msgs::Twist());
  ros::Duration(brake_time).sleep();

  const double target_rad = target_deg * M_PI / 180.0;
  log_line("turn_start target_rad=" + std::to_string(target_rad));
  turn_angle_imu(cmd_pub, rate, target_rad, angular_speed, min_angular_speed, timeout_s, kp);
  log_line("turn_done yaw_now=" + std::to_string(imu_yaw_corrected()));

  // Final brake
  cmd_pub.publish(geometry_msgs::Twist());
  ros::Duration(brake_time).sleep();

  ROS_INFO("imu_turn180_node finished.");
  log_line("node_finished");
  return 0;
}
