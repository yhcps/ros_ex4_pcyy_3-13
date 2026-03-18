# obstacle_avoid

最小可运行的 ROS1 激光避障节点：订阅 `sensor_msgs/LaserScan`，发布 `geometry_msgs/Twist` 到底盘。

## 功能概述
- 读取激光雷达 `/scan`（可通过参数/launch remap）。
- 计算前方扇区最小距离，若小于安全距离则原地转向，否则直行。
- 参数可调：安全距离、前进速度、转向速度、前方扇区角度。

## 主要参数（对应 `obstacle_avoid.launch`）
- `scan_topic` (默认 `/scan`)
- `cmd_vel_topic` (默认 `/cmd_vel`)
- `safe_dist` 安全距离阈值，米（默认 0.6）
- `forward_speed` 直行线速度，米/秒（默认 0.2）
- `turn_speed` 转向角速度，弧度/秒（默认 0.6）
- `front_angle_deg` 前方扇区角度，度（默认 60）

## 构建
在 `avoid` 工作区下：
```bash
cd /home/bcsh/workspace/ex4pcyy/avoid
catkin_make
source devel/setup.bash
```

## 运行
确保底盘 bringup 已在另一终端运行（例如 `roslaunch upros_bringup bringup_w2a.launch`，且有激光 `/scan` 和底盘订阅 `/cmd_vel`）。

### 方式 1：直接 launch
```bash
roslaunch obstacle_avoid obstacle_avoid.launch
```
可选参数：
```bash
roslaunch obstacle_avoid obstacle_avoid.launch \
  scan_topic:=/scan \
  cmd_vel_topic:=/cmd_vel \
  safe_dist:=0.7 \
  forward_speed:=0.25 \
  turn_speed:=0.8 \
  front_angle_deg:=80
```

### 方式 2：一键脚本
在工作区根目录：
```bash
./run_avoid.sh          # 当前终端运行避障节点（要求已启动 bringup）
./run_all_avoid.sh      # 自动新终端跑 bringup，本终端跑避障
```

## 逻辑简介
- 从雷达数据中提取前方扇区（|angle| < front_angle/2）的最小距离。
- 若 `min_front < safe_dist`：发布纯角速度 `turn_speed`（左转），线速度 0。
- 否则：发布前进速度 `forward_speed`，角速度 0。
- 未收到激光前，不动。

## 依赖
- ROS1 Noetic，依赖 `roscpp`, `sensor_msgs`, `geometry_msgs`。
