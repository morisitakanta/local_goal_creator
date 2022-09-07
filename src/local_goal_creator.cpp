#include "local_goal_creator/local_goal_creator.h"

LocalGoalCreator::LocalGoalCreator():
    nh_(),
    private_nh_("~")
{
    private_nh_.param("hz", hz_, 10);

    checkpoint_sub_ = nh_.subscribe("/checkpoint", 1, &LocalGoalCreator::checkpoint_callback, this);
    node_edge_sub_ = nh_.subscribe("/node_edge_map", 1, &LocalGoalCreator::node_edge_callback, this);
    current_pose_sub_ = nh_.subscribe("/current_pose", 1, &LocalGoalCreator::current_pose_callback, this);
    local_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/local_goal", 1);
}

void LocalGoalCreator::checkpoint_callback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    ROS_INFO("checkpoint_callback");
}

void LocalGoalCreator::node_edge_callback(const amsl_navigation_msgs::NodeEdgeMap::ConstPtr& msg)
{
    ROS_INFO("node_edge_callback");
}

void LocalGoalCreator::current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_INFO("current_pose_callback");
}

void LocalGoalCreator::process()
{
    ros::Rate rate(hz_);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}