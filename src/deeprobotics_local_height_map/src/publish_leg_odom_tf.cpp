// message_transformer/qnx2ros.cpp 会发布 /leg_odom
// 该文件：订阅 /leg_odom，实时更新 odom -> base_link 的TF变换

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class LegOdomTfPublisher
{
public:
  LegOdomTfPublisher()
    : private_nh_("~")
  {
    private_nh_.param("odom_topic", odom_topic_, std::string("/leg_odom"));
    private_nh_.param("parent_frame", parent_frame_, std::string("odom")); // 世界坐标系
    private_nh_.param("child_frame", child_frame_, std::string("base_link")); // 机器人坐标系

    odom_sub_ = nh_.subscribe(odom_topic_, 10, &LegOdomTfPublisher::odomCallback, this);

    ROS_INFO_STREAM("publish_leg_odom_tf listening on " << odom_topic_
                    << ", publishing " << parent_frame_ << " -> " << child_frame_);
  }

private:
  /**
   * @brief 里程计位姿回调函数，将位姿消息转换为TF变换并广播
   * 
   * 该函数接收一个包含位姿和协方差的ROS消息，从中提取位置和四元数姿态信息，
   * 构建一个tf::Transform对象。然后根据消息头中的frame_id确定父坐标系
   * （若消息中为空则使用默认的parent_frame_），最后通过TF广播器将此变换
   * 作为带时间戳的TF变换发布出去。
   * 
   * @param msg 指向geometry_msgs::PoseWithCovarianceStamped消息的常量智能指针，
   *           包含位姿信息、协方差以及时间戳和坐标系ID。
   */
  void odomCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
  {
    tf::Transform transform;
    // 提取位置信息
    transform.setOrigin(tf::Vector3(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z));

    // 提取四元数姿态信息
    tf::Quaternion rotation(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    transform.setRotation(rotation);

    // 确定父坐标系
    const std::string parent = msg->header.frame_id.empty() ? parent_frame_ : msg->header.frame_id;

    // 广播TF：将组装好的 Transform 打包成 StampedTransform（带有时间戳的变换），时间戳直接服用消息的 header.stamp 保证时间同步，父坐标系使用 parent，子坐标系使用 child_frame_
    broadcaster_.sendTransform(
      tf::StampedTransform(transform, msg->header.stamp, parent, child_frame_));
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odom_sub_;
  tf::TransformBroadcaster broadcaster_;

  std::string odom_topic_;
  std::string parent_frame_;
  std::string child_frame_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_leg_odom_tf");
  LegOdomTfPublisher publisher;
  ros::spin();
  return 0;
}