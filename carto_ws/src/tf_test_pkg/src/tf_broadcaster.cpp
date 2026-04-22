#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
 
    tf::TransformBroadcaster broadcaster;
    tf::Transform base_link2base_laser;
    base_link2base_laser.setOrigin( tf::Vector3(0.1, 0.0, 0.2) );
    base_link2base_laser.setRotation( tf::Quaternion(0, 0, 0, 1) );
 
    while(n.ok())
    {
        broadcaster.sendTransform( tf::StampedTransform( base_link2base_laser, ros::Time::now(), "base_link", "base_laser"));
        //broadcaster.sendTransform(
        //    tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0, 0), tf::Vector3(1, 0.0, 0) ),
        //    ros::Time::now(), "base link", "base laser") );
        loop_rate.sleep();
    }
    return 0;
}