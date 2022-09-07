#ifndef __LOCAL_GOAL_CREATOR_H__
#define __LOCAL_GOAL_CREATOR_H__

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include "amsl_navigation_msgs/NodeEdgeMap.h"
#include "amsl_navigation_msgs/Node.h"
#include "amsl_navigation_msgs/Edge.h"
#include <geometry_msgs/PoseStamped.h>

class LocalGoalCreator
{
    public:
        LocalGoalCreator();
        void process();

    private:
        // callbacks
        void checkpoint_callback(const std_msgs::Int32MultiArray::ConstPtr& msg);
        void node_edge_callback(const amsl_navigation_msgs::NodeEdgeMap::ConstPtr& msg);
        void current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        // other functions

        // private params
        int hz_;

        // other params

        // ros
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber checkpoint_sub_;
        ros::Subscriber node_edge_sub_;
        ros::Subscriber current_pose_sub_;
        ros::Publisher local_goal_pub_;
};

#endif // __LOCAL_GOAL_CREATOR_H__