# 基础依赖

安装常用编译工具和依赖：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  python3-catkin-tools python3-rosdep \
  libeigen3-dev libboost-all-dev libpcl-dev \
  libopencv-dev libyaml-cpp-dev libtbb-dev qtbase5-dev \
  ros-noetic-pcl-ros ros-noetic-cv-bridge ros-noetic-filters \
  ros-noetic-tf ros-noetic-tf2-ros ros-noetic-tf-conversions \
  ros-noetic-eigen-conversions \
  ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher \
  ros-noetic-rviz ros-noetic-octomap-msgs ros-noetic-costmap-2d
rosdep update
```

# 编译 voa-lite

```bash
cd /home/ysc/lite_cog/voa-lite
source /home/ysc/lite_cog/drivers/realsense_ws/devel/setup.bash
source /home/ysc/lite_cog/transfer/devel/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic --skip-keys="gazebo"
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF
catkin build
source devel/setup.bash
```

# 使用 voa

```bash
cd /home/ysc/lite_cog/voa-lite
chmod +x ./start_voa_lite_realsense.sh
./start_voa_lite_realsense.sh
```

<mark>需要保证transfer服务是关闭状态，脚本已自动实现</mark>

脚本会处理这两个常见服务名：

```bash
sudo systemctl stop transfer.service
sudo systemctl stop message_transformer.service
```

如果手柄控制没有效果，试一下：

```bash
rostopic pub /cmd_vel geometry_msgs/Twist \
"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
-r 10
```

重点关注：

```bash
rostopic info /cmd_vel_corrected
应该是：
========== /cmd_vel_corrected ==========
Publishers:
 * /safety_node

Subscribers:
 * /ros2qnx
 
rostopic info /cmd_vel
========== /cmd_vel ==========
  Publishers: 手柄/导航/上层控制节点
  Subscribers: /safety_node
  不能出现: /ros2qnx
```



# 启动后的检查

```bash
cd /home/ysc/lite_cog/voa-lite
./check_voa_safety_topics.sh
```

期望结果：
```bash
========== systemd transfer services ==========
transfer.service                 inactive
message_transformer.service     inactive
========== ROS nodes ==========
/safety_node
/ros2qnx
/qnx2ros
/nx2app
/sensor_checker
========== /cmd_vel ==========
Publishers:
 * /手柄或导航节点

Subscribers:
 * /safety_node
 
========== /cmd_vel_corrected ==========
Publishers:
 * /safety_node

Subscribers:
 * /ros2qnx
========== /cmd_vel_disabled_by_voa ==========
Publishers: None

Subscribers:
 * /ros2qnx
========== VOA safety status sample ==========
data: "state=pass vx=0 vy=0 wz=0 collision_time=-1 scale=1"
```







重点看下面三个 topic 的订阅关系：

```bash
rostopic info /cmd_vel
rostopic info /cmd_vel_corrected
rostopic info /cmd_vel_disabled_by_voa
```

