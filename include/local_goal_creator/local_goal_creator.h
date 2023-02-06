#ifndef __LOCAL_GOAL_CREATOR_H__
#define __LOCAL_GOAL_CREATOR_H__

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class LocalGoalCreator
{
    public:
        LocalGoalCreator();
        void process();

    private:
        // callbacks
        void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        void global_path_callback(const nav_msgs::Path::ConstPtr& msg);

        // other functions
        geometry_msgs::PoseStamped get_local_goal(geometry_msgs::PoseStamped &current_pose, nav_msgs::Path &global_path, int &local_goal_index);
        bool reached_goal(int index, geometry_msgs::PoseStamped current_pose);

        // private params
        int hz_;
        bool use_covariance_;
        std::string robot_name_;
        double local_goal_dist_;
        std::string local_goal_frame_id_;

        // other params
        bool current_pose_updated_;
        geometry_msgs::PoseStamped current_pose_;
        bool global_path_updated_;
        nav_msgs::Path global_path_;
        int local_goal_index_;
        geometry_msgs::PoseStamped local_goal_;

        // ros
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber pose_sub_;
        ros::Subscriber global_path_sub_;
        ros::Publisher local_goal_pub_;

        // tf
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener *tf_listener_;
};

#endif // __LOCAL_GOAL_CREATOR_H__