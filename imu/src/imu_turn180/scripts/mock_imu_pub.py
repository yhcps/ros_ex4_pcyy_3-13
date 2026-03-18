#!/usr/bin/env python3
"""
Mock IMU publisher that can simulate a real IMU by integrating commanded angular.z on /cmd_vel.

Behavior:
 - Subscribes to /cmd_vel (or configured topic) and integrates angular.z to update yaw.
 - Publishes IMU messages to /imu/data at a fixed rate.

Usage examples:
  # publish static yaw=0 (no cmd_vel input)
  rosrun imu_turn180 mock_imu_pub.py _yaw_deg:=0

  # listen to /cmd_vel and update yaw accordingly
  rosrun imu_turn180 mock_imu_pub.py _listen_cmd_vel:=true

Params (rosparam / remap):
  ~yaw_deg: initial yaw in degrees (used only if not listening to cmd_vel)
  ~topic: imu topic (default /imu/data)
  ~rate: publish rate (default 50.0)
  ~cmd_vel_topic: topic to listen for angular velocity (default /cmd_vel)
  ~listen_cmd_vel: if true, subscribe to cmd_vel and integrate angular.z
"""
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
import math
import threading

try:
    from tf.transformations import quaternion_from_euler
except Exception:
    # fallback implementation
    def quaternion_from_euler(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return (x, y, z, w)


class MockImu:
    def __init__(self):
        rospy.init_node('mock_imu_pub')
        self.yaw_deg = rospy.get_param('~yaw_deg', 0.0)
        self.topic = rospy.get_param('~topic', '/imu/data')
        self.rate_hz = rospy.get_param('~rate', 50.0)
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.listen_cmd_vel = rospy.get_param('~listen_cmd_vel', False)

        self._lock = threading.Lock()
        self._yaw = math.radians(self.yaw_deg)
        self._last_time = rospy.Time.now().to_sec()
        self._ang_z = 0.0

        self.pub = rospy.Publisher(self.topic, Imu, queue_size=10)
        if self.listen_cmd_vel:
            rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_cb)

    def cmd_vel_cb(self, msg: Twist):
        with self._lock:
            self._ang_z = msg.angular.z

    def step(self):
        now = rospy.Time.now().to_sec()
        with self._lock:
            dt = now - self._last_time
            # integrate angular z to update yaw
            self._yaw += self._ang_z * dt
            self._yaw = ((self._yaw + math.pi) % (2 * math.pi)) - math.pi
            self._last_time = now

            q = quaternion_from_euler(0.0, 0.0, self._yaw)

        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = 'imu_link'
        imu.orientation = Quaternion(*q)
        imu.orientation_covariance = [-1.0] + [0.0]*8
        # we don't simulate angular velocity/acceleration beyond orientation
        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = self._ang_z
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 0.0

        self.pub.publish(imu)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo('mock_imu_pub: publishing to %s @ %.1f Hz, listen_cmd_vel=%s',
                      self.topic, self.rate_hz, self.listen_cmd_vel)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


def main():
    m = MockImu()
    m.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
