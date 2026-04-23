#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "body_to_move");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string parent_frame;
  std::string body_frame;
  std::string move_frame;
  double publish_rate;

  private_nh.param<std::string>("parent_frame", parent_frame, "camera_init");
  private_nh.param<std::string>("body_frame", body_frame, "body");
  private_nh.param<std::string>("move_frame", move_frame, "move");
  private_nh.param<double>("publish_rate", publish_rate, 100.0);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  tf2_ros::TransformBroadcaster tf_broadcaster;

  ros::Rate rate(publish_rate);
  while (ros::ok())
  {
    geometry_msgs::TransformStamped parent_to_body;
    try
    {
      parent_to_body = tf_buffer.lookupTransform(parent_frame, body_frame, ros::Time(0), ros::Duration(0.05));
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(1.0, "body_to_move lookup failed: %s", ex.what());
      rate.sleep();
      continue;
    }

    tf2::Quaternion q_parent_body;
    tf2::fromMsg(parent_to_body.transform.rotation, q_parent_body);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q_parent_body).getRPY(roll, pitch, yaw);

    tf2::Quaternion q_parent_move;
    q_parent_move.setRPY(0.0, 0.0, yaw);
    q_parent_move.normalize();

    tf2::Quaternion q_body_move = q_parent_body.inverse() * q_parent_move;
    q_body_move.normalize();

    geometry_msgs::TransformStamped body_to_move;
    body_to_move.header.stamp = ros::Time::now();
    body_to_move.header.frame_id = body_frame;
    body_to_move.child_frame_id = move_frame;
    body_to_move.transform.translation.x = 0.0;
    body_to_move.transform.translation.y = 0.0;
    body_to_move.transform.translation.z = 0.0;
    body_to_move.transform.rotation = tf2::toMsg(q_body_move);

    tf_broadcaster.sendTransform(body_to_move);
    ROS_INFO_ONCE("body_to_move started successfully!");
    rate.sleep();
  }

  return 0;
}
