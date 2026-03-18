# imu_turn180

基于 IMU 的闭环原地自转 180° 节点（ROS1）。参考 `ex2pcyy` 的 `ros_walk1x1.cpp`，简化为只做一次角度旋转，可通过参数调整目标角度与速度。

## 功能
- 订阅 IMU (`sensor_msgs/Imu`)，可选零点校准（默认以上电朝向为 0°）。
- 按目标角度（默认 180°）闭环控制角速度，限制最小/最大角速度，避免静摩擦。
- 支持参数：目标角度、角速度上限、最小角速度、超时时间、比例增益、刹车时间、话题名等。

## 依赖
- ROS1（已测试接口：`roscpp`, `geometry_msgs`, `sensor_msgs`, `tf2`, `tf2_ros`）。

## 代码结构
- `src/imu_turn180.cpp`：节点实现。
- `CMakeLists.txt` / `package.xml`：catkin 包配置。

## 编译
将本包放入你的 catkin 工作空间 `src` 目录，然后在工作空间根目录执行：

```bash
catkin_make
source devel/setup.bash
```

编译成功后会得到可执行文件 `imu_turn180_node`。

## 运行示例

```bash
rosrun imu_turn180 imu_turn180_node \
  _target_deg:=180 \
  _angular_speed:=0.8 \
  _min_angular_speed:=0.3 \
  _timeout_s:=12.0 \
  _kp:=1.6 \
  _brake_time:=0.3 \
  _imu_topic:=/imu/data \
  _cmd_vel_topic:=/cmd_vel \
  _imu_offset_enable:=true
```

关键参数说明：
- `~target_deg`：目标转角（度），默认 180。
- `~angular_speed`：最大角速度上限（rad/s）。
- `~min_angular_speed`：最小角速度，用于克服静摩擦。
- `~timeout_s`：超时（秒），超过后停止并退出。
- `~kp`：比例增益（默认 1.6，末端留更大控制量）。
- `~brake_time`：起转/结束前的刹车时间（秒），用于清除残余速度。
- `~imu_topic` / `~cmd_vel_topic`：IMU 与速度控制话题。
- `~imu_offset_enable`：是否按上电姿态做零点校准；若需要指定初始朝向，可传 `~imu_initial_yaw`（弧度）。

## 运行流程简述
1. 等待 IMU 数据（默认 5 秒超时）。
2. 如启用零点校准，捕获初始朝向作为 0°。
3. 预刹停，随后按闭环控制转到目标角度（容差约 1°）。
4. 刹停结束。

## 调试提示
- 若底盘起步困难，可提高 `~min_angular_speed`；若抖动，适当降低 `~kp` 或上限 `~angular_speed`。
- 若 IMU 姿态存在固定偏移，可传 `~imu_initial_yaw`（弧度）作为零点。
- 如需开环按时间转动，可在 `src/imu_turn180.cpp` 中仿照 `ex2pcyy` 的开环分支添加（当前实现仅闭环 IMU）。
