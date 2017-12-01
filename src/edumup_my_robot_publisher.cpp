#include <string>
#include <ros/ros.h>
#include <edumip_msgs/EduMipState.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

class EduMipRobotPub
{
public:
  EduMipRobotPub();

private:
  ros::NodeHandle n;
  edumip_msgs::EduMipState EduMipState;
  ros::Subscriber sub;
  ros::Publisher joint_publisher;

  void EduMipStateCallback(const edumip_msgs::EduMipState::ConstPtr& EduMipState);
};

EduMipRobotPub::EduMipRobotPub()
{
  joint_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  sub = n.subscribe("edumip/state", 100, &EduMipRobotPub::EduMipStateCallback, this);
}

void EduMipRobotPub::EduMipStateCallback(const edumip_msgs::EduMipState::ConstPtr& EduMipState)
{
  static double angle = 0.0;
  static sensor_msgs::JointState   js;
  static tf::TransformBroadcaster  tf_broadcaster;
  static tf::Transform             robot_trans;

  robot_trans.setOrigin( tf::Vector3( EduMipState->body_frame_northing, -EduMipState->body_frame_easting, 0.034));

  tf::Quaternion q;
  q.setRPY(0.0 , EduMipState->theta, -EduMipState->body_frame_heading);
  robot_trans.setRotation(q);

  // get joint state info
  js.name.resize(2);
  js.position.resize(2);
  js.name[0]      ="jointL";
  js.name[1]      ="jointR";

  js.header.stamp = ros::Time::now();
  js.position[0]  = EduMipState->wheel_angle_L;
  js.position[1]  = EduMipState->wheel_angle_R;

  joint_publisher.publish(js);
  tf_broadcaster.sendTransform(tf::StampedTransform(robot_trans,  ros::Time::now(), "world", "edumip_body"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "edumip_my_robot_publisher");
  EduMipRobotPub eduMipRobotPub;
  ros::spin();
  return 0;
}
