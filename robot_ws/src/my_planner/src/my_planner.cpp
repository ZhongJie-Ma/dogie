#include "my_planner.h"
#include <pluginlib/class_list_macros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

PLUGINLIB_EXPORT_CLASS( my_planner::MyPlanner, nav_core::BaseLocalPlanner)

namespace my_planner 
{
    MyPlanner::MyPlanner()
    {
        setlocale(LC_ALL,"");
    }
    MyPlanner::~MyPlanner()
    {}

    tf::TransformListener* tf_listener_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    void MyPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_INFO("路径追踪开启！");
        tf_listener_ = new tf::TransformListener();
        costmap_ros_ = costmap_ros;
    }

    std::vector<geometry_msgs::PoseStamped> global_plan_;
    int target_index_;
    bool pose_adjusting_;
    bool goal_reached_; 
    bool MyPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        target_index_ = 0;
        global_plan_ = plan;
        pose_adjusting_ = false;
        goal_reached_ = false;
        return true;
    }

    bool MyPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        // 获取代价地图的数据
        costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
        unsigned char* map_data = costmap->getCharMap();
        unsigned int size_x = costmap->getSizeInCellsX();
        unsigned int size_y = costmap->getSizeInCellsY();

        for (unsigned int y = 0; y < size_y; y++)
        {
            for (unsigned int x = 0; x < size_x; x++)
            {
                int map_index = y * size_x + x;
                unsigned char cost = map_data[map_index];               // 从代价地图数据取值
            }
        }

        // 在代价地图上遍历导航路径点
        for(int i=0;i<global_plan_.size();i++)
        {
            geometry_msgs::PoseStamped pose_odom;
            global_plan_[i].header.stamp = ros::Time(0);
            tf_listener_->transformPose("camera_init",global_plan_[i],pose_odom);
            double odom_x = pose_odom.pose.position.x;
            double odom_y = pose_odom.pose.position.y;

            double origin_x = costmap->getOriginX();
            double origin_y = costmap->getOriginY();
            double local_x = odom_x - origin_x;
            double local_y = odom_y - origin_y;
            int x = local_x / costmap->getResolution();
            int y = local_y / costmap->getResolution();
            // 检测前方路径点是否在禁行区域或者障碍物里
            if(i >= target_index_ && i < target_index_ + 10)
            {
                int map_index = y * size_x + x;
                unsigned char cost = map_data[map_index];
                if(cost >= 253)
                    return false;
            }
        }


        int final_index = global_plan_.size()-1;
        geometry_msgs::PoseStamped pose_final;
        global_plan_[final_index].header.stamp = ros::Time(0);
        tf_listener_->transformPose("body_2d",global_plan_[final_index],pose_final);
        if(pose_adjusting_ == false)
        {
            double dx = pose_final.pose.position.x;
            double dy = pose_final.pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            if(dist < 0.2)
                pose_adjusting_ = true;
        }
        if(pose_adjusting_ == true)
        {
            double final_yaw = tf::getYaw(pose_final.pose.orientation);
            ROS_INFO("调整最终姿态，final_yaw = %.2f",final_yaw);
            cmd_vel.linear.x = 0.0;
            double angular_speed = final_yaw * 0.5;
            double min_angular = 0.3;
            double max_angular = 1.0;
            if (std::fabs(angular_speed) < min_angular)
                angular_speed = std::copysign(min_angular, final_yaw);
            if (std::fabs(angular_speed) > max_angular)
                angular_speed = std::copysign(max_angular, final_yaw);
            cmd_vel.angular.z = angular_speed;
            if(abs(final_yaw) < 0.6)
            {
                goal_reached_ = true;
                ROS_INFO("到达终点！");
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
            }
            return true;
        }

        geometry_msgs::PoseStamped target_pose;
        for(int i=target_index_;i<global_plan_.size();i++)
        {
            geometry_msgs::PoseStamped pose_base;
            global_plan_[i].header.stamp = ros::Time(0);
            tf_listener_->transformPose("body_2d",global_plan_[i],pose_base);
            double dx = pose_base.pose.position.x;
            double dy = pose_base.pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);

            if (dist > 0.2) 
            {
                target_pose = pose_base;
                target_index_ = i;
                ROS_INFO("选择第 %d 个路径点作为临时目标，距离=%.2f",target_index_,dist);
                break;
            }

            if(i == global_plan_.size()-1)
                target_pose = pose_base; 
        }
        cmd_vel.linear.x = target_pose.pose.position.x * 2.5;
        cmd_vel.angular.z = target_pose.pose.position.y * 3.5;
        return true;
    }
    bool MyPlanner::isGoalReached()
    {
        return goal_reached_;
    }
}
