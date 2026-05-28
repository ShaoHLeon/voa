/**
 * @file safety.cpp
 * @brief 机器狗安全限速与拦截节点 (VOA - Velocity Obstacle Avoidance)
 *
 * @details
 * 本节点作为机器人运动控制链路中的“安全网关”，位于底层通信节点 (ros2qnx) 之前。
 * 它的核心职责是：
 * 1. 订阅高层导航或遥控指令（如 /cmd_vel）。
 * 2. 订阅局部地形高度图（GridMap），用于感知周围环境。
 * 3. 对输入指令进行安全校验：包括速度硬限幅、指令超时检测、地图超时检测。
 * 4. 将经过安全过滤的指令发布到输出话题（ /cmd_vel_corrected ）。
 *
 * 架构说明：
 * 在 enable_safety=true 时，底层 ros2qnx 节点会被 launch 文件通过 <remap> 
 * 强制订阅 /cmd_vel_disabled_by_voa，从而物理断开原始控制链路。
 * 所有指令必须经过本 safety_node 过滤，通过 /cmd_vel_corrected 下发，确保机器人不会因
 * 失控指令或传感器丢失而发生碰撞。
 *
 * @author shl
 * @date 2026-05-25 （初代版本，timerCallback）第一版保守停车逻辑，不会绕障
        /cmd_vel 原始速度
                ↓
        cmd 是否超时？
                ↓
        地图是否有效？
                ↓
        速度限幅
                ↓
        预测未来轨迹
                ↓
        检查轨迹上的机器人足迹范围内是否有高障碍物
                ↓
        安全则发布 /cmd_vel_corrected，“pass 正常放行：使用加速度限制，速度慢慢升上去；slow_down 遇到障碍减速：不限制减速，立即按安全比例降速；stop_obstacle / stop_map_timeout / stop_cmd_timeout：不限制，直接发 0。”
 * @date 2026-05-26 （第二代版本，timerCallback）
        /cmd_vel -> 预测当前速度轨迹
          -> 无障碍：正常放行
          -> 有障碍：尝试替代速度候选
              -> 左绕安全：发布左绕速度
              -> 右绕安全：发布右绕速度
              -> 都不安全：再减速或停车
              替代速度代价 = 和原始 vx 的差距 + 和原始 vy 的差距 + 和原始 yaw 的差距 + 是否切换绕障方向的惩罚
 */

#include <algorithm>
#include <sstream>
#include <string>

#include <geometry_msgs/Twist.h>
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <cmath>
#include <vector>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class SafetyNode
{
public:
  SafetyNode()
    : private_nh_("~")
  {
    // ========================= 参数初始化 ========================
    // 话题名称
    private_nh_.param("grid_map_topic", grid_map_topic_, std::string("/elevation_mapping/elevation_map"));
    private_nh_.param("grid_map_layer", grid_map_layer_, std::string("elevation"));
    private_nh_.param("cmd_vel_input_topic", cmd_vel_input_topic_, std::string("/cmd_vel"));
    private_nh_.param("cmd_vel_output_topic", cmd_vel_output_topic_, std::string("/cmd_vel_corrected"));
    private_nh_.param("publish_debug_status", publish_debug_status_, true);
    private_nh_.param("debug_status_topic", debug_status_topic_, std::string("/voa/safety_status"));

    // 安全逻辑控制参数
    private_nh_.param("stop_when_map_timeout", stop_when_map_timeout_, true); // 地图超时是否 stop
    private_nh_.param("map_timeout", map_timeout_, 0.5); // 地图超时阈值
    private_nh_.param("cmd_timeout", cmd_timeout_, 0.5); // 控制指令超时阈值

    // 速度限制，安全边界
    private_nh_.param("vel_x_max", vel_x_max_, 0.5); // x 前进最大线速度
    private_nh_.param("vel_x_back_max", vel_x_back_max_, 0.5); // 后退最大线速度
    private_nh_.param("vel_y_max", vel_y_max_, 0.6); // 横向最大线速度
    private_nh_.param("vel_yaw_max", vel_yaw_max_, 1.2); // 最大偏航角速度
    private_nh_.param("rate", rate_, 50.0); // 控制循环频率
    private_nh_.param("enable_acceleration_limit", enable_acceleration_limit_, true);
    private_nh_.param("acc_x", acc_x_, 0.6);
    private_nh_.param("acc_y", acc_y_, 0.5);
    private_nh_.param("acc_yaw", acc_yaw_, 0.8);

    // 坐标系
    private_nh_.param("grid_map_frame", grid_map_frame_, std::string("odom"));
    private_nh_.param("robot_base_frame", robot_base_frame_, std::string("base_link"));

    // 障碍物检测参数
    private_nh_.param("enable_obstacle_check", enable_obstacle_check_, true);
    private_nh_.param("obstacle_ground_height", obstacle_ground_height_, 0.0);
    private_nh_.param("obstacle_height_threshold", obstacle_height_threshold_, 0.12);
    private_nh_.param("obstacle_min_points", obstacle_min_points_, 3);
    private_nh_.param("use_dynamic_ground_height", use_dynamic_ground_height_, true);
    private_nh_.param("ground_estimation_radius", ground_estimation_radius_, 0.35);
    private_nh_.param("ground_min_points", ground_min_points_, 5);
    private_nh_.param("directional_obstacle_check", directional_obstacle_check_, true);
    private_nh_.param("motion_direction_deadband", motion_direction_deadband_, 0.03);
    private_nh_.param("directional_min_projection", directional_min_projection_, 0.05);

    // 避障控制参数
    private_nh_.param("enable_slow_down", enable_slow_down_, true);
    private_nh_.param("stop_time_threshold", stop_time_threshold_, 0.35);
    private_nh_.param("slow_down_time_threshold", slow_down_time_threshold_, 1.0);
    private_nh_.param("min_slow_down_scale", min_slow_down_scale_, 0.25);
    private_nh_.param("enable_avoidance_candidates", enable_avoidance_candidates_, true);
    private_nh_.param("avoidance_lateral_speed", avoidance_lateral_speed_, 0.20);
    private_nh_.param("avoidance_forward_scale", avoidance_forward_scale_, 0.35);
    private_nh_.param("avoidance_yaw_scale", avoidance_yaw_scale_, 0.0);
    private_nh_.param("avoidance_min_input_speed", avoidance_min_input_speed_, 0.05);
    private_nh_.param("enable_avoidance_direction_hold", enable_avoidance_direction_hold_, true);
    private_nh_.param("avoidance_direction_hold_time", avoidance_direction_hold_time_, 0.5);
    private_nh_.param("avoidance_forward_cost_weight", avoidance_forward_cost_weight_, 1.0);
    private_nh_.param("avoidance_lateral_cost_weight", avoidance_lateral_cost_weight_, 1.0);
    private_nh_.param("avoidance_yaw_cost_weight", avoidance_yaw_cost_weight_, 0.3);
    private_nh_.param("avoidance_direction_switch_cost", avoidance_direction_switch_cost_, 0.30);

    // 预测轨迹参数
    private_nh_.param("collision_check_time", collision_check_time_, 1.0);
    private_nh_.param("collision_check_dt", collision_check_dt_, 0.10);

    // 机器人尺寸
    private_nh_.param("len_base_front", len_base_front_, 0.5);
    private_nh_.param("len_base_back", len_base_back_, 0.62);
    private_nh_.param("len_base_side", len_base_side_, 0.3);
    private_nh_.param("safety_margin", safety_margin_, 0.10);

    // ========================= 通信接口初始化 ========================
    // 订阅 ："/elevation_mapping/elevation_map"
    map_sub_ = nh_.subscribe(grid_map_topic_, 1, &SafetyNode::mapCallback, this);
    // 订阅 ："/cmd_vel"
    cmd_sub_ = nh_.subscribe(cmd_vel_input_topic_, 1, &SafetyNode::cmdCallback, this);
    // 发布 ："/cmd_vel_corrected"
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_output_topic_, 1);
    if (publish_debug_status_) {
      debug_status_pub_ = nh_.advertise<std_msgs::String>(debug_status_topic_, 1);
    }

    // ========================= 定时器初始化 ========================
    // 使用定时器作为控制主循环，确保在无指令或无地图时也能持续发布零速度指令
    const double timer_period = 1.0 / std::max(rate_, 1.0); // 防止除0
    // 周期性定时器，每个 timer_period s 自动调用一次 timerCallback
    timer_ = nh_.createTimer(ros::Duration(timer_period), &SafetyNode::timerCallback, this);

    ROS_INFO_STREAM("safety_node input cmd: " << cmd_vel_input_topic_);
    ROS_INFO_STREAM("safety_node output cmd: " << cmd_vel_output_topic_);
    ROS_INFO_STREAM("safety_node grid map: " << grid_map_topic_ << ", layer: " << grid_map_layer_);
  }

private:
  /**
    * @brief 限制 value 在 min_value 和 max_value 之间
    * 
    * @param value 要限制的值
    * @param min_value 最小值 
    * @param max_value 最大值
    * 
    * @return 限制后的 value
   */
  static double clamp(double value, double min_value, double max_value)
  {
    return std::max(min_value, std::min(value, max_value));
  }


  /**
   * @brief 接收并处理网格地图(GridMap)消息的回调函数
   * 当 /elevation_mapping/elevation_map 收到新消息时，ROS 自动调用该函数。
   * 
   * 该函数在接收到新的网格地图消息时被调用。它会更新本地存储的最新地图数据，
   * 记录接收时间，并检查地图中是否包含所需的特定数据层。
   * 
   * @param msg 指向接收到的grid_map_msgs::GridMap消息的常量智能指针
   */
  void mapCallback(const grid_map_msgs::GridMapConstPtr& msg)
  {
    // 保存最新地图，取msg指针指向的实际 GripMap 对象
    latest_map_ = *msg;
    last_map_time_ = ros::Time::now();
    has_map_ = true; // 收到地图数据

    // 在容器中查找 grid_map_layer_ 是否存在
    has_grid_layer_ =
      std::find(msg->layers.begin(), msg->layers.end(), grid_map_layer_) != msg->layers.end();

    // 如果地图中没有所需的图层，输出警告信息
    if (!has_grid_layer_) {
      ROS_WARN_THROTTLE(2.0, "GridMap does not contain required layer: %s", grid_map_layer_.c_str());
    }
    if (has_grid_layer_) {
        // 将接收到的 GridMap 转换为 grid_map::GridMap 对象。latest_grid_map_：输出参数，将消息中的数据（图层矩阵、坐标系、时间戳等）解析并存储到这个 C++ 对象中
        // grid_map::GridMapRosConverter::fromMessage(*msg, latest_grid_map_);
        if (!grid_map::GridMapRosConverter::fromMessage(*msg, latest_grid_map_)) {
          // 如果消息格式异常、图层数据不完整，转换可能失败。转换失败时继续用旧地图做避障是不安全的，所以这里把 has_grid_layer_ 置回 false，让主循环走“地图无效则停车”。
          has_grid_layer_ = false;
          ROS_WARN_THROTTLE(2.0, "Failed to convert GridMap message.");
        }
    }
  }

  /**
   * @brief 速度指令回调函数，用于接收并保存最新的速度控制指令
   * 当输入速度话题收到 geometry_msgs::Twist 消息时调用
   * 
   * @param msg 指向 geometry_msgs::Twist 消息的常量智能指针，包含线速度和角速度指令
   */
  void cmdCallback(const geometry_msgs::TwistConstPtr& msg)
  {
    // 保存最新的速度指令
    latest_cmd_ = *msg;
    last_cmd_time_ = ros::Time::now();
    has_cmd_ = true; // 收到指令
  }

  void timerCallback(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();
    // 创建一个 geometry_msgs::Twist 对象，用于存储安全过滤后的速度指令
    geometry_msgs::Twist safe_cmd; // 默认“零速度指令”

    // -------------- 控制指令超时保护 -------------
    if (!has_cmd_ || (now - last_cmd_time_).toSec() > cmd_timeout_) {
      // 发布零速度指令
      publishStatus("stop_cmd_timeout", safe_cmd);
      // cmd_pub_.publish(safe_cmd);
      publishCommand(safe_cmd);
      return;
    }

    // -------------- 地图有效性保护 --------------
    // 如果地图未收到或地图图层不存在，或者地图已超时，发布零速度指令
    if (stop_when_map_timeout_ &&
        (!has_map_ || !has_grid_layer_ || (now - last_map_time_).toSec() > map_timeout_)) {
      ROS_WARN_THROTTLE(1.0, "No valid elevation map recently. Publishing zero velocity.");
      publishStatus("stop_map_timeout", safe_cmd);
      // cmd_pub_.publish(safe_cmd);
      publishCommand(safe_cmd);
      return;
    }

    // -------------- 使用最新的 cmd_vel，并限制速度 --------------
    safe_cmd = latest_cmd_;
    applyVelocityLimits(safe_cmd);

    // -------------- 检查轨迹是否安全 --------------
    // 初代逻辑
    // if (enable_obstacle_check_ && isTrajectoryUnsafe(safe_cmd)) {
    //     ROS_WARN_THROTTLE(1.0, "Obstacle detected on predicted trajectory. Publishing zero velocity.");
    //     geometry_msgs::Twist stop_cmd;
    //     cmd_pub_.publish(stop_cmd); // 发布零速度指令
    //     return;
    // }
    if (enable_obstacle_check_) {
      double collision_time = 0.0;
      
      // 原速度不安全
      if (estimateCollisionTime(safe_cmd, collision_time)) {
        // 优先寻找绕行路径
        geometry_msgs::Twist alternative_cmd; // 存储备用的绕行速度指令
        double alternative_collision_time = 0.0; // 存储备用指令的碰撞时间
        if (findSafeAlternativeCommand(safe_cmd, now, alternative_cmd, alternative_collision_time)) {
          ROS_WARN_THROTTLE(1.0, "Obstacle ahead. Publishing avoidance candidate velocity.");
          publishStatus("avoidance_candidate", alternative_cmd, alternative_collision_time, 1.0);
          publishCommand(alternative_cmd);
          return;
        }

        // 如果没有找到绕行路径，则根据碰撞时间决定是否减速
        if (!enable_slow_down_ || collision_time <= stop_time_threshold_) { // 如果不需要减速或碰撞时间小于等于停车时间阈值
          ROS_WARN_THROTTLE(1.0, "Obstacle too close. Publishing zero velocity.");
          geometry_msgs::Twist stop_cmd;
          publishStatus("stop_obstacle", stop_cmd, collision_time, 0.0);
          // cmd_pub_.publish(stop_cmd);
          publishCommand(stop_cmd);
          return;
        }

        const double scale = computeSlowDownScale(collision_time);
        scaleVelocity(safe_cmd, scale);

        ROS_WARN_THROTTLE(1.0, "Obstacle ahead. Scaling velocity by %.2f.", scale);
        publishStatus("slow_down", safe_cmd, collision_time, scale);
        // cmd_pub_.publish(safe_cmd);
        publishCommand(safe_cmd);
        return;
      }
    }

    applyAccelerationLimits(safe_cmd, now);
    publishStatus("pass", safe_cmd);
    // cmd_pub_.publish(safe_cmd);
    publishCommand(safe_cmd);
  }


  /**
   * @brief 对（x、y、yaw 三个方向）速度指令施加最大速度限制
   * 
   * 该函数用于将输入的速度指令中的线速度和角速度分量限制在允许的最大值范围内。
   * 对于前进和后退的X轴线速度，分别使用不同的最大值进行限制；
   * 对于Y轴线速度和Z轴角速度，则使用对称的最大值进行限制。
   * 
   * @param cmd 输入/输出的速度指令引用。其linear.x、linear.y和angular.z字段将被修改，
   *            以确保它们不超过对应的最大速度限制。
   */
  void applyVelocityLimits(geometry_msgs::Twist& cmd) const
  {
    cmd.linear.x = clamp(cmd.linear.x, -vel_x_back_max_, vel_x_max_);
    cmd.linear.y = clamp(cmd.linear.y, -vel_y_max_, vel_y_max_);
    cmd.angular.z = clamp(cmd.angular.z, -vel_yaw_max_, vel_yaw_max_);
  }

  /**
   * @brief 根据预测的碰撞时间，计算速度的缩放比例（减速因子）
   * 
   * @details 速度缩放因子遵循以下规则：
   *          1. 碰撞时间 > 减速阈值：安全，不减速 (返回 1.0)
   *          2. 碰撞时间 < 停车阈值：危险，急刹车 (返回 0.0)
   *          3. 介于两者之间：线性插值，从最小缩放比例平滑过渡到 1.0
   * 
   * @param collision_time 预测到碰撞发生的时间（秒）
   * @return double 速度缩放因子，范围 [0.0, 1.0]
   */
  double computeSlowDownScale(double collision_time) const
  {
    // 距离障碍物较远，无需减速
    if (collision_time >= slow_down_time_threshold_) {
      return 1.0;
    }
    // 靠近障碍物，需要急刹车
    if (collision_time <= stop_time_threshold_) {
      return 0.0;
    }

    // 减速缓冲区域：介于停车和全速之间，进行线性减速
    // 计算减速缓冲区的时间跨度 (停车阈值 到 减速阈值 之间的时长)
    const double range = std::max(0.001, slow_down_time_threshold_ - stop_time_threshold_);
    // 计算当前碰撞时间在缓冲区内的相对位置比例
    const double ratio = (collision_time - stop_time_threshold_) / range;
    return min_slow_down_scale_ + ratio * (1.0 - min_slow_down_scale_);
  }

  void scaleVelocity(geometry_msgs::Twist& cmd, double scale) const
  {
    scale = clamp(scale, 0.0, 1.0);
    cmd.linear.x *= scale;
    cmd.linear.y *= scale;
    cmd.angular.z *= scale;
  }

  /**
   * @brief 发布机器人状态
   */
  void publishStatus(const std::string& state,
                     const geometry_msgs::Twist& cmd,
                     double collision_time = -1.0,
                     double scale = 1.0)
  {
    if (!publish_debug_status_) {
      return;
    }

    std_msgs::String msg;
    std::ostringstream stream;
    stream << "state=" << state
           << " vx=" << cmd.linear.x
           << " vy=" << cmd.linear.y
           << " wz=" << cmd.angular.z
           << " collision_time=" << collision_time
           << " scale=" << scale;

    msg.data = stream.str();
    debug_status_pub_.publish(msg);
  }

  void publishCommand(const geometry_msgs::Twist& cmd)
  {
    cmd_pub_.publish(cmd);
    last_published_cmd_ = cmd;
    last_publish_time_ = ros::Time::now();
    has_last_published_cmd_ = true;
  }

  /**
   * @brief 限制变量从先前值变化到目标值的最大变化率，防止速度突变
   * 
   * 该函数用于计算在给定最大变化率和时间步长的情况下，变量从先前值向目标值
   * 过渡时的实际值。如果变化率的限制无效（max_rate <= 0）或时间步长无效
   * （dt <= 0），则直接返回目标值。否则，变化量将被限制在 [-max_delta, max_delta] 
   * 的范围内，以确保变化率不超过 max_rate。
   * 
   * @param previous   变量的先前值
   * @param target     变量的目标值
   * @param max_rate   允许的最大变化率（单位：值/秒）。若小于等于0，则不进行限制
   * @param dt         时间步长（单位：秒）。若小于等于0，则不进行限制
   * @return double    经过变化率限制后计算得到的当前实际值
   */
  static double limitRate(double previous, double target, double max_rate, double dt)
  {
    if (max_rate <= 0.0 || dt <= 0.0) {
      return target;
    }

    const double max_delta = max_rate * dt;
    const double delta = clamp(target - previous, -max_delta, max_delta);
    return previous + delta;
  }

  /**
   * @brief 对速度指令施加加速度限制，确保机器人运动平滑且不超过最大加速度
   * 
   * @param cmd 当前待发送的速度指令，包含线速度和角速度，函数会对其原地进行修改
   * @param now 当前时间戳，用于计算与上一次发送指令的时间间隔
   * 
   * @note 如果未启用加速度限制或没有上一次发送的指令记录，则直接返回；
   *       如果计算出的时间间隔无效（非有限数、非正数或大于1.0秒），
   *       则使用控制频率的倒数作为默认时间间隔。
   */
  void applyAccelerationLimits(geometry_msgs::Twist& cmd, const ros::Time& now)
  {
    // 如果没有开启加速度限制功能，或没有历史指令记录
    if (!enable_acceleration_limit_ || !has_last_published_cmd_) {
      return;
    }

    double dt = (now - last_publish_time_).toSec(); // 计算时间间隔
    // 如果时间间隔无效（非有限数、非正数或大于1.0秒），则使用控制频率的倒数作为默认时间间隔
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 1.0) {
      dt = 1.0 / std::max(rate_, 1.0);
    }

    cmd.linear.x = limitRate(last_published_cmd_.linear.x, cmd.linear.x, acc_x_, dt);
    cmd.linear.y = limitRate(last_published_cmd_.linear.y, cmd.linear.y, acc_y_, dt);
    cmd.angular.z = limitRate(last_published_cmd_.angular.z, cmd.angular.z, acc_yaw_, dt);
  }

   struct VelocityCandidate
  {
    geometry_msgs::Twist cmd;
    int direction = 0; // 1: left, -1: right, 0: no lateral preference
  };



  /**
   * @brief 计算给定速度候选者的代价值。
   * 
   * 该函数通过比较候选速度与期望速度之间的差异来计算代价。代价包括前向、侧向和偏航角
   * 三个维度上的绝对差异加权求和。如果当前处于保持激活状态且候选方向发生了改变，
   * 则会额外增加方向切换的惩罚代价。
   * 
   * @param candidate 待评估的速度候选者，包含候选速度指令和方向信息。
   * @param desired_cmd 期望的速度指令，作为计算差异的基准。
   * @param hold_active 布尔值，指示是否处于保持激活状态；若为true且方向切换，将增加额外代价。
   * @return double 计算得出的总代价值。
   */
  double computeCandidateCost(const VelocityCandidate& candidate,
                              const geometry_msgs::Twist& desired_cmd,
                              bool hold_active) const
  {
    double cost = 0.0;

    cost += avoidance_forward_cost_weight_ *
            std::abs(candidate.cmd.linear.x - desired_cmd.linear.x); // 
    cost += avoidance_lateral_cost_weight_ *
            std::abs(candidate.cmd.linear.y - desired_cmd.linear.y);
    cost += avoidance_yaw_cost_weight_ *
            std::abs(candidate.cmd.angular.z - desired_cmd.angular.z);

    if (hold_active &&
        candidate.direction != 0 &&
        candidate.direction != last_avoidance_direction_) {
      cost += avoidance_direction_switch_cost_;
    }

    return cost;
  }


  // /**
  //  * @brief 生成速度候选并将其添加到候选列表中
  //  * 
  //  * 该函数会根据给定的线速度和角速度创建一个速度候选对象，
  //  * 对其应用速度限制，然后将处理后的候选对象推入候选列表中。
  //  * 
  //  * @param candidates 速度候选对象的引用向量，新的候选将被添加到此列表的末尾
  //  * @param vx X轴方向上的线速度
  //  * @param vy Y轴方向上的线速度
  //  * @param wz Z轴方向上的角速度
  //  */
  // void pushVelocityCandidate(std::vector<geometry_msgs::Twist>& candidates,
  //                            double vx,
  //                            double vy,
  //                            double wz) const
  // {
  //   geometry_msgs::Twist candidate;
  //   candidate.linear.x = vx;
  //   candidate.linear.y = vy;
  //   candidate.angular.z = wz;
  //   applyVelocityLimits(candidate);
  //   candidates.push_back(candidate);
  // }

  /**
   * @brief 添加速度候选项到候选列表中
   * 
   * 构造一个速度候选对象，设置其线速度、角速度和方向信息，
   * 对其应用速度限制，最后将其压入候选列表中。
   * 
   * @param candidates  速度候选项的列表，新候选项将被添加到此列表中
   * @param vx          x轴方向的线速度分量
   * @param vy          y轴方向的线速度分量
   * @param wz          z轴方向的角速度分量
   * @param direction   运动方向标识
   */
  void pushVelocityCandidate(std::vector<VelocityCandidate>& candidates,
                             double vx,
                             double vy,
                             double wz,
                             int direction) const
  {
    VelocityCandidate candidate;
    candidate.cmd.linear.x = vx;
    candidate.cmd.linear.y = vy;
    candidate.cmd.angular.z = wz;
    candidate.direction = direction;
    applyVelocityLimits(candidate.cmd);
    candidates.push_back(candidate);
  }

  /**
  * @brief 寻找安全的避障替代速度指令
  * 
  * 当机器人当前期望的速度指令可能导致碰撞时，该函数会根据预设的
  * 横向速度、前进速度缩放比例和角速度缩放比例，生成一系列替代速度候选。
  * 随后依次评估这些候选速度，如果某个候选速度不会导致碰撞（即估计碰撞时间失败），
  * 则将其作为安全的替代指令返回。
  * 
  * @param desired_cmd 期望的速度指令，作为生成候选速度的参考基准。
  * @param now 当前时间戳，用于计算与上一次发送指令的时间间隔。
  * @param alternative_cmd [out] 找到的安全替代速度指令，仅在函数返回 true 时有效。
  * @param alternative_collision_time [out] 替代速度的碰撞时间，若找到安全替代指令，
  *                                         该值被设置为 collision_check_time_。
  * 
  * @return true 成功找到不会发生碰撞的安全替代速度指令；
  * @return false 未找到安全替代指令（可能因为避障功能未启用或输入速度过小）。
  */
  bool findSafeAlternativeCommand(const geometry_msgs::Twist& desired_cmd,
                                  const ros::Time& now,
                                  geometry_msgs::Twist& alternative_cmd,
                                  double& alternative_collision_time)
  {
    // 如果没有启用避障绕行功能，则直接返回
    if (!enable_avoidance_candidates_) {
      return false;
    }

    // 如果输入速度小于最小输入速度阈值，说明机器人移动缓慢，则直接返回，使用原始速度
    const double input_speed = std::hypot(desired_cmd.linear.x, desired_cmd.linear.y);
    if (input_speed < avoidance_min_input_speed_) {
      return false;
    }

    // std::vector<geometry_msgs::Twist> candidates; // 存储所有可能的避障绕行速度候选
    std::vector<VelocityCandidate> candidates; // 存储所有可能的避障绕行速度候选

    const double lateral_speed = std::abs(avoidance_lateral_speed_); // 避障绕行（横向）速度的绝对值
    const double forward_vx = desired_cmd.linear.x * avoidance_forward_scale_; // 避障绕行（前进）速度（原来速度的35%）
    const double yaw_rate = desired_cmd.angular.z * avoidance_yaw_scale_; // 避障绕行角速度（原来速度的0%）

    // 根据desired_cmd.linear.y的正负，生成不同的速度候选
    // if (desired_cmd.linear.y >= 0.0) {
    //   pushVelocityCandidate(candidates, forward_vx, lateral_speed, yaw_rate); 
    //   pushVelocityCandidate(candidates, forward_vx, -lateral_speed, yaw_rate);
    //   pushVelocityCandidate(candidates, 0.0, lateral_speed, 0.0);
    //   pushVelocityCandidate(candidates, 0.0, -lateral_speed, 0.0);
    // } else {
    //   pushVelocityCandidate(candidates, forward_vx, -lateral_speed, yaw_rate);
    //   pushVelocityCandidate(candidates, forward_vx, lateral_speed, yaw_rate);
    //   pushVelocityCandidate(candidates, 0.0, -lateral_speed, 0.0);
    //   pushVelocityCandidate(candidates, 0.0, lateral_speed, 0.0);
    // }

    const int preferred_direction = desired_cmd.linear.y < 0.0 ? -1 : 1; // 根据desired_cmd.linear.y的正负，确定首选的避障方向
    const bool hold_active =
      enable_avoidance_direction_hold_ &&
      last_avoidance_direction_ != 0 &&
      (now - last_avoidance_time_).toSec() <= avoidance_direction_hold_time_; // 检查是否需要保持避障方向

    const int first_direction = hold_active ? last_avoidance_direction_ : preferred_direction; // 如果需要保持避障方向，则使用上一次的避障方向；否则使用首选的避障方向
    const int second_direction = -first_direction; // 第二个避障方向与第一个方向相反

    const double first_lateral = first_direction * lateral_speed; // 根据第一个避障方向计算横向速度
    const double second_lateral = second_direction * lateral_speed;

    pushVelocityCandidate(candidates, forward_vx, first_lateral, yaw_rate, first_direction);
    pushVelocityCandidate(candidates, forward_vx, second_lateral, yaw_rate, second_direction);
    pushVelocityCandidate(candidates, 0.0, first_lateral, 0.0, first_direction);
    pushVelocityCandidate(candidates, 0.0, second_lateral, 0.0, second_direction);

    // 未使用代价函数
    // for (std::size_t i = 0; i < candidates.size(); ++i) {
    //   double candidate_collision_time = 0.0;
    //   if (!estimateCollisionTime(candidates[i].cmd, candidate_collision_time)) {
    //     alternative_cmd = candidates[i].cmd;
    //     last_avoidance_direction_ = candidates[i].direction; // 记录前一次避障方向
    //     last_avoidance_time_ = now; // 记录前一次避障时间
    //     alternative_collision_time = collision_check_time_;
    //     return true;
    //   }
    // }
    // return false;

    bool found_safe_candidate = false; // 标记是否找到安全候选
    double best_cost = 0.0; // 记录最佳候选的代价
    VelocityCandidate best_candidate; // 记录最佳候选

    for (std::size_t i = 0; i < candidates.size(); ++i) {
      double candidate_collision_time = 0.0; // 记录当前候选的碰撞时间

      // estimateCollisionTime 返回 true 表示会碰撞，所以这里跳过不安全候选。
      if (estimateCollisionTime(candidates[i].cmd, candidate_collision_time)) {
        continue;
      }

      // 使用代价函数计算当前候选的代价
      const double cost = computeCandidateCost(candidates[i], desired_cmd, hold_active);
      if (!found_safe_candidate || cost < best_cost) {
        found_safe_candidate = true;
        best_cost = cost;
        best_candidate = candidates[i];
      }
    }

    if (!found_safe_candidate) {
      return false;
    }

    alternative_cmd = best_candidate.cmd;
    alternative_collision_time = collision_check_time_;
    last_avoidance_direction_ = best_candidate.direction;
    last_avoidance_time_ = now;
    return true;

    
  }


  /** old
   * @brief 检查给定的速度指令在预测轨迹上是否会导致碰撞，判断轨迹是否不安全
   * 
   * 该函数通过获取机器人当前位姿，并根据输入的速度指令预测未来一段时间内的运动轨迹。
   * 在预测轨迹的每个时间步长上，检查机器人足迹是否与障碍物发生碰撞。
   * 如果无法获取机器人的当前位姿，则默认轨迹是不安全的。
   * 
   * @param cmd 输入的速度指令，包含线速度和角速度 (geometry_msgs::Twist)
   * @return bool 如果轨迹不安全（发生碰撞或无法获取位姿），返回 true；否则返回 false
   */
  // bool isTrajectoryUnsafe(const geometry_msgs::Twist& cmd)
  /**
   * @brief 预测在给定速度指令下，机器人与障碍物发生碰撞的时间
   * 
   * @details 通过获取机器人当前位姿，结合输入的速度指令，在时间轴上进行前向运动学推演。
   * 在每个推演时刻，利用 footprintHasObstacle 检查底盘足迹是否触碰障碍物。
   * 若检测到碰撞，则计算并返回碰撞发生的时间；若整条预测轨迹均安全，则返回无碰撞。
   * 
   * @param cmd             当前输入的速度指令 (线速度 x, y 和角速度 z)
   * @param collision_time  [输出参数] 预测到碰撞发生的时间 (秒)。仅在返回 true 时有效
   * @return true           预测轨迹上存在碰撞风险
   * @return false          预测轨迹安全，无碰撞风险
   */
  bool estimateCollisionTime(const geometry_msgs::Twist& cmd, double& collision_time)
  {
    collision_time = 0.0;
    // 获取当前机器人位姿
    tf::StampedTransform transform;
    try {
        // 查询从 地图坐标系 到 机器人底盘坐标系 的变换，ros::Time(0) 表示获取最新的变换
        tf_listener_.lookupTransform(grid_map_frame_, robot_base_frame_, ros::Time(0), transform);
    } catch (const tf::TransformException& ex) { // 如果 TF 树未连通或数据延迟，捕获异常并打印警告（节流1秒）
        ROS_WARN_THROTTLE(1.0, "Cannot lookup robot pose for safety check: %s", ex.what());
        // 感知不到自身位置是极度危险的，默认返回 true (不安全)，阻断运动
        return true;
    }

    // 提取当前机器人的 X, Y 坐标和偏航角(Yaw)
    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double yaw = tf::getYaw(transform.getRotation());

    // 如果 TF 错、地图没跟随机器人、或者 frame 配错，机器人位置可能落在地图外。此时如果继续检测，CircleIterator 可能找不到有效点，反而误判为安全。安全节点应该保守处理：机器人不在地图里，就停车。
    if (!latest_grid_map_.isInside(grid_map::Position(x, y))) {
      ROS_WARN_THROTTLE(1.0, "Robot pose is outside the elevation map. Publishing zero velocity.");
      collision_time = 0.0;
      return true;
    }

    double ground_height = obstacle_ground_height_;
    if (!estimateGroundHeight(x, y, ground_height)) {
      ROS_WARN_THROTTLE(1.0, "Not enough ground points. Falling back to configured ground height.");
    }

    // 仿真参数计算
    // 仿真步长，取配置值与0.02秒中的较大值，防止步长过小导致计算量爆炸
    const double dt = std::max(collision_check_dt_, 0.02);
    // 仿真步数 = 总仿真时间 / 步长，至少为1步
    const int steps = std::max(1, static_cast<int>(collision_check_time_ / dt));

    // 前向仿真循环：沿时间轴推演机器人位置
    for (int i = 0; i <= steps; ++i) {
        // 在当前推演位置，检查底盘是否压到障碍物
        if (footprintHasObstacle(x, y, yaw, ground_height, cmd.linear.x, cmd.linear.y)) {
          collision_time = i * dt;
          return true;
        }

        // 运动学模型推演：根据当前速度和航向，计算下一个 dt 时刻的位置
        x += (cmd.linear.x * std::cos(yaw) - cmd.linear.y * std::sin(yaw)) * dt;
        y += (cmd.linear.x * std::sin(yaw) + cmd.linear.y * std::cos(yaw)) * dt;
        yaw += cmd.angular.z * dt;
    }

    return false;
  }


  /**
   * @brief 估计给定机器人位置周围的地面高度
   * 地面高度只需要机器人附近一圈点，不需要扫完整地图。
   * 
   * 该函数根据配置决定是否使用动态地面高度估计。如果使用动态估计，它会在以机器人
   * 为中心的指定半径内，从栅格地图中提取有效的高度数据，并计算其中位数作为地面高度。
   * 如果不使用动态估计，或者在指定半径内有效数据点不足，则返回预设的静态地面高度。
   * 
   * @param robot_x 机器人的X坐标
   * @param robot_y 机器人的Y坐标
   * @param ground_height [out] 估计得到的地面高度引用，用于存储结果
   * @return bool 如果成功使用动态数据估计了地面高度则返回true；
   *             如果使用了静态默认高度（因禁用动态估计或有效点数不足）则返回false
   */
  bool estimateGroundHeight(double robot_x, double robot_y, double& ground_height) const
  {
    // 如果禁用动态地面高度估计，直接返回预设的静态地面高度
    if (!use_dynamic_ground_height_) {
      ground_height = obstacle_ground_height_;
      return true;
    }

    // 局部区域高度采样
    std::vector<double> heights; // 用于存储搜索范围内所有有效栅格的高度值
    // const double radius_sq = ground_estimation_radius_ * ground_estimation_radius_;

    // 遍历最新的栅格地图中的每一个栅格
    const grid_map::Position center(robot_x, robot_y);

    // 旧逻辑：遍历整张地图后再用半径过滤，计算量随地图面积增长。
    // for (grid_map::GridMapIterator iterator(latest_grid_map_); !iterator.isPastEnd(); ++iterator) {
    for (grid_map::CircleIterator iterator(latest_grid_map_, center, ground_estimation_radius_);
        !iterator.isPastEnd(); ++iterator) {
      // 检查当前栅格在指定图层中是否有效
      if (!latest_grid_map_.isValid(*iterator, grid_map_layer_)) {
        continue;
      }

      grid_map::Position position;
      latest_grid_map_.getPosition(*iterator, position);

      // CircleIterator 已经只遍历半径内栅格，这里的距离过滤不再需要。
      // 计算当前栅格到机器人位置的平移量
      // const double dx = position.x() - robot_x;
      // const double dy = position.y() - robot_y;

      // 判断当前栅格是否在搜索半径内
      // if (dx * dx + dy * dy > radius_sq) { // 超过则跳过
      //   continue;
      // }

      // 符合条件的栅格，将其高度值加入数组
      heights.push_back(latest_grid_map_.at(grid_map_layer_, *iterator));
    }

    // 有效性检验
    // 如果在搜索范围内找到有效高度点数量低于设定的最小点数阈值
    if (static_cast<int>(heights.size()) < ground_min_points_) {
      ground_height = obstacle_ground_height_; // 使用预设的静态地面高度
      return false; // 动态估算失败
    }

    // 统计求值（中位数滤波）
    // 对采集到的高度值进行升序排序
    std::sort(heights.begin(), heights.end());
    // 取排序后的中间值作为地面高度
    ground_height = heights[heights.size() / 2];
    return true;
  }

  
  /**
  * @brief 检测机器人底盘足迹范围内是否存在障碍物
  * 
  * 通过遍历高程图栅格，将点转换至机器人底盘坐标系，结合包围盒过滤、
  * 方向性投影过滤及高度阈值判断，检测底盘范围内是否存在障碍物。
  * 
  * @param robot_x 机器人在世界坐标系下的 X 坐标
  * @param robot_y 机器人在世界坐标系下的 Y 坐标
  * @param robot_yaw 机器人在世界坐标系下的偏航角（弧度）
  * @param ground_height 当前地面的基准高度
  * @param cmd_vx 机器人的 X 方向指令速度
  * @param cmd_vy 机器人的 Y 方向指令速度
  * @return true 底盘范围内存在障碍物（障碍物点数达到阈值）
  * @return false 底盘范围内无障碍物或障碍物点数未达阈值
  */
  bool footprintHasObstacle(double robot_x, double robot_y, double robot_yaw, double ground_height, double cmd_vx, double cmd_vy) const
  {
    int obstacle_count = 0; // 记录地盘范围内障碍物点的数量
    // 预计算三角函数
    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);
    const double footprint_radius =
      std::hypot(std::max(len_base_front_, len_base_back_) + safety_margin_,
                len_base_side_ + safety_margin_);
    const grid_map::Position center(robot_x, robot_y);

    // 1.  遍历高程图每一个栅格
    // 旧逻辑：遍历整张地图，再通过矩形足迹过滤。
    // for (grid_map::GridMapIterator iterator(latest_grid_map_); !iterator.isPastEnd(); ++iterator) {
    for (grid_map::CircleIterator iterator(latest_grid_map_, center, footprint_radius);
        !iterator.isPastEnd(); ++iterator) {
        // 跳过无效数据
        if (!latest_grid_map_.isValid(*iterator, grid_map_layer_)) {
            continue;
        }

        // 获取当前栅格在世界坐标系下的二维坐标
        grid_map::Position position;
        latest_grid_map_.getPosition(*iterator, position);

        // 2. 坐标变换：将地图中的点转换到以机器人中心为原点的底盘坐标系
        // 平移变换
        const double dx = position.x() - robot_x;
        const double dy = position.y() - robot_y;

        // 旋转矩阵逆变换 (等价于将世界系的点旋转到机器人系)
        const double local_x = cos_yaw * dx + sin_yaw * dy;
        const double local_y = -sin_yaw * dx + cos_yaw * dy;

        // 3. 包围盒过滤 (AABB碰撞检测的简化版)
        const double front_limit = len_base_front_ + safety_margin_;
        const double back_limit = len_base_back_ + safety_margin_;
        const double side_limit = len_base_side_ + safety_margin_;
        // 检查该点是否超出了机器人底盘的矩形边界，超出则无需判断高度，直接跳过
        if (local_x < -back_limit || local_x > front_limit) {
          continue;
        }
        if (std::abs(local_y) > side_limit) {
          continue;
        }

        // 方向性障碍物检测
        // 开启后，只有处于机器人运动方向前方的障碍物才会被记入，避免侧方/后方的正常地形误触
        if (directional_obstacle_check_) {
          // 计算当前指令的平移速度标量
          const double translational_speed = std::hypot(cmd_vx, cmd_vy);

          // 判断是否处于有效运动状态
          if (translational_speed > motion_direction_deadband_) {
            // 计算运动方向的单位向量
            const double dir_x = cmd_vx / translational_speed;
            const double dir_y = cmd_vy / translational_speed;
            
            // 计算当前栅格在运动方向上的投影
            const double projection = local_x * dir_x + local_y * dir_y; // 障碍物在运动方向上距离机器人的前向距离

            // 投影过滤
            // 如果投影小于设定的最小前向投影距离，说明障碍物在侧方或后方
            // 忽略这些点，避免机器人在横向移动或原地旋转时被侧后方的点误刹停
            if (projection < directional_min_projection_) { 
              continue;
            }
          }
        }

        // 4. 高度阈值判断
        // 对于落在底盘矩形内的点，读取其高程值
        const double height = latest_grid_map_.at(grid_map_layer_, *iterator);
        // 如果该点的高度减去地面高度，超过了设定的障碍物高度阈值（例如高于地面5cm视为障碍）
        if (height - ground_height > obstacle_height_threshold_) {
            ++obstacle_count; // 累加障碍物点
            // 只要障碍物点数达到设定的最小判定点数（防止单点噪点误触发），立即判定为碰撞
            if (obstacle_count >= obstacle_min_points_) {
                return true;
            }
        }
    }

    // 遍历完地图，底盘内的障碍物点数未达阈值，判定为安全
    return false;
  }

  // ========================= 成员变量 ================================
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_; // 私有命名空间，用于读取节点内部参数

  ros::Subscriber map_sub_; // 地图订阅
  ros::Subscriber cmd_sub_; // 控制指令订阅
  ros::Publisher cmd_pub_; // 安全控制指令发布
  ros::Publisher debug_status_pub_; // 调试状态发布
  ros::Timer timer_; // 定时器

  // 话题与图层名称
  std::string grid_map_topic_;
  std::string grid_map_layer_;
  std::string cmd_vel_input_topic_;
  std::string cmd_vel_output_topic_;
  std::string debug_status_topic_; // 调试状态话题

  // 状态标志位
  bool stop_when_map_timeout_ = true;
  bool has_map_ = false; // 是否收到地图
  bool has_grid_layer_ = false; // 地图是否包含所需要的图层
  bool has_cmd_ = false; // 是否收到指令
  bool publish_debug_status_ = true; // 是否发布调试状态

  // 超时与频率
  double map_timeout_ = 0.5;
  double cmd_timeout_ = 0.5;
  double rate_ = 50.0;

  // 速度限制
  double vel_x_max_ = 0.5;
  double vel_x_back_max_ = 0.5;
  double vel_y_max_ = 0.6;
  double vel_yaw_max_ = 1.2;
  // 加速度限制
  bool enable_acceleration_limit_ = true;
  double acc_x_ = 0.6;
  double acc_y_ = 0.5;
  double acc_yaw_ = 0.8;

  // 时间戳
  ros::Time last_map_time_;
  ros::Time last_cmd_time_;

  // 高程图以及控制指令
  grid_map_msgs::GridMap latest_map_;
  geometry_msgs::Twist latest_cmd_; // 存储从 ROS 话题接收并转换后的最新控制指令数据
  geometry_msgs::Twist last_published_cmd_; // 存储上一次发布的控制指令数据
  bool has_last_published_cmd_ = false; // 是否有上一次发布的控制指令数据
  ros::Time last_publish_time_;
  grid_map::GridMap latest_grid_map_; // 高程地图 C++ 对象缓存。存储从 ROS 话题接收并转换后的最新栅格地图数据，供碰撞检测遍历


  // 坐标系
  std::string grid_map_frame_; // 高程地图坐标系 odom
  std::string robot_base_frame_; // 机器人底盘坐标系 base_link

  // 障碍物检测
  bool enable_obstacle_check_ = true; // 是否启用障碍物检测
  double obstacle_ground_height_ = 0.0; // 地面基准高程
  double obstacle_height_threshold_ = 0.12; // 障碍物高度阈值
  int obstacle_min_points_ = 3; // 障碍物判定所需的最小点数
  bool use_dynamic_ground_height_ = true; // 是否使用动态地面高度估计
  double ground_estimation_radius_ = 0.35; // 动态地面高度估计半径 (单位：m)。在机器人当前位置附近取一圈有效栅格，用中位数作为当前地面高度
  int ground_min_points_ = 5; // 动态地面高度估计所需的最小栅格数
  bool directional_obstacle_check_ = true; // 是否启用方向性障碍物检测
  double motion_direction_deadband_ = 0.03; // 机器狗运动方向死区。当机器狗运动方向与前进方向夹角小于该值时，不进行方向性障碍物检测
  double directional_min_projection_ = 0.05; // 方向性障碍物检测的最小投影距离。当机器人运动方向与障碍物方向夹角小于该值时，认为障碍物在运动方向上
  // 减速 / 刹停策略
  bool enable_slow_down_ = true; // 是否启用减速/刹停策略
  double stop_time_threshold_ = 0.35; // 预测碰撞时间小于该值时，直接停车
  double slow_down_time_threshold_ = 1.0; // 预测碰撞时间小于该值时开始减速
  double min_slow_down_scale_ = 0.25; // 减速时的最小速度比例
  // 替代速度候选 / 简单绕障
  bool enable_avoidance_candidates_ = true;
  double avoidance_lateral_speed_ = 0.20;
  double avoidance_forward_scale_ = 0.35;
  double avoidance_yaw_scale_ = 0.0;
  double avoidance_min_input_speed_ = 0.05;
  bool enable_avoidance_direction_hold_ = true; // 是否启用方向保持
  double avoidance_direction_hold_time_ = 0.5; // 方向保持时间阈值
  double avoidance_forward_cost_weight_ = 1.0; // 前向避障代价权重
  double avoidance_lateral_cost_weight_ = 1.0; // 侧向避障代价权重
  double avoidance_yaw_cost_weight_ = 0.3; // 偏航避障代价权重
  double avoidance_direction_switch_cost_ = 0.30; // 方向切换代价
  int last_avoidance_direction_ = 0; // 0: 不绕障，1: 左绕障，-1: 右绕障
  ros::Time last_avoidance_time_; // 上一次绕障时间

  // 轨迹前向仿真参数
  double collision_check_time_ = 1.0; // 前向仿真总时长 (单位：秒)。推演机器人未来 1 秒内的运动轨迹
  double collision_check_dt_ = 0.10; // 前向仿真步长 (单位：秒)。每 0.1 秒（100ms）离散采样一个点检查碰撞，1秒将采样10个点

  // 机器狗尺寸
  // 定义机器人底盘在 X 轴和 Y 轴上占据的范围，用于 footprintHasObstacle 函数中的包围盒(AABB)碰撞检测
  double len_base_front_ = 0.5; // 底盘中心到前边缘的距离 m
  double len_base_back_ = 0.62; // 底盘中心到后边缘的距离
  double len_base_side_ = 0.3; // 底盘中心到侧边缘的距离
  double safety_margin_ = 0.10; // 安全距离，用于在障碍物检测时，在障碍物周围增加额外的安全距离，避免机器狗碰到障碍物边缘

  tf::TransformListener tf_listener_; // TF 监听器。用于实时查询机器人当前在地图中的位姿 (X, Y, Yaw)
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "safety_node");
  SafetyNode safety_node;
  ros::spin();
  return 0;
}