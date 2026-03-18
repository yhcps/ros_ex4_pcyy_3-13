# obstacle_avoid

最小可运行的 ROS1 激光避障节点：订阅 `sensor_msgs/LaserScan`，发布 `geometry_msgs/Twist` 到底盘，可选订阅碰撞开关 `/robot/bump_sensor` 以优先避障。

## 功能概述
- 读取激光雷达 `/scan`（可通过参数 / launch remap）。
- 碰撞开关 `/robot/bump_sensor`（`std_msgs/Int32MultiArray`，前三位依次为左前 / 中前 / 右前），优先级最高：触发即后退并转向脱离。
- 计算前方扇区最小距离，若小于安全距离则原地转向，否则直行。
- 参数可调：安全距离、前进速度、转向速度、前方扇区角度；可配置无雷达 fallback。

## 主要参数（对应 `obstacle_avoid.launch`）
- `scan_topic` (默认 `/scan`)
- `cmd_vel_topic` (默认 `/cmd_vel`)
- `safe_dist` 安全距离阈值，米（默认 0.6）
- `forward_speed` 直行线速度，米/秒（默认 0.2）
- `turn_speed` 转向角速度，弧度/秒（默认 0.6）
- `front_angle_deg` 前方扇区角度，度（默认 60）
- `enable_bump` 是否启用碰撞开关优先逻辑（默认 true）
- `bump_topic` 碰撞开关话题（默认 `/robot/bump_sensor`，`std_msgs/Int32MultiArray`，前三位左/中/右）
- `bump_timeout` 碰撞信号保持时长秒，超时后自动恢复激光逻辑（默认 0.6）
- `bump_back_speed` 碰撞触发时的后退速度（默认 -0.08）
- `bump_turn_speed` 碰撞触发时的转向角速度基准，左右开关触发分别向相反方向旋转（默认 0.5）

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
  front_angle_deg:=80 \
  enable_bump:=true \
  bump_topic:=/robot/bump_sensor
```

### 方式 2：一键脚本
在工作区根目录：
```bash
./run_avoid.sh          # 当前终端运行避障节点（要求已启动 bringup）
./run_all_avoid.sh      # 自动新终端跑 bringup，本终端跑避障
```

## 逻辑简介
- 碰撞优先：若 `enable_bump` 且最近 `bump_timeout` 内检测到任意前方开关触发，则发布后退速度 `bump_back_speed`；若左触发则右转，右触发则左转，左右同时略偏左；中间仅触发则直后退。
- 否则，从雷达数据中提取前方扇区（|angle| < front_angle/2）的最小距离。
- 若 `min_front < safe_dist`：发布纯角速度 `turn_speed`（左转），线速度 0。
- 否则：发布前进速度 `forward_speed`，角速度 0。
- 未收到激光前，不动；碰撞优先时仍可触发后退。

## 依赖
- ROS1 Noetic，依赖 `roscpp`, `sensor_msgs`, `geometry_msgs`, `std_msgs`。
