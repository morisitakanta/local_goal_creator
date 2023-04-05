#include "local_goal_creator/local_goal_creator.h"

LocalGoalCreator::LocalGoalCreator() : nh_(),
                                       private_nh_("~")
{
    private_nh_.param("hz", hz_, 10);
    private_nh_.param("use_covariance", use_covariance_, false);
    private_nh_.param("robot_name", robot_name_, std::string("robot_0"));
    private_nh_.param("local_goal_dist", local_goal_dist_, 5.0);
    private_nh_.param("local_goal_frame_id", local_goal_frame_id_, std::string("base_link"));

    current_pose_updated_ = false;
    global_path_updated_ = false;
    local_goal_index_ = 0;

    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

    // ros
    if (use_covariance_)
        pose_sub_ = nh_.subscribe("/" + robot_name_ + "/amcl_pose", 1, &LocalGoalCreator::amcl_pose_callback, this);
    else
        pose_sub_ = nh_.subscribe("/" + robot_name_ + "/pose", 1, &LocalGoalCreator::pose_callback, this);
    global_path_sub_ = nh_.subscribe("/" + robot_name_ + "/global_path", 1, &LocalGoalCreator::global_path_callback, this);
    local_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/" + robot_name_ + "/local_goal", 1);
}

void LocalGoalCreator::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ROS_INFO("------------------------------------");
    ROS_INFO("pose_callback");
    current_pose_ = *msg;
    current_pose_updated_ = true;
}

void LocalGoalCreator::amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    ROS_INFO("------------------------------------");
    ROS_INFO("amcl_pose_callback");
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
    current_pose_updated_ = true;
}

void LocalGoalCreator::global_path_callback(const nav_msgs::Path::ConstPtr &msg)
{
    ROS_INFO("------------------------------------");
    ROS_INFO("global_path_callback");
    global_path_ = *msg;
    global_path_updated_ = true;
}

geometry_msgs::PoseStamped LocalGoalCreator::get_local_goal(geometry_msgs::PoseStamped &current_pose, nav_msgs::Path &global_plan, int &local_goal_index)
{
    // ROS_INFO("get_local_goal");
    double current_local_goal_x = global_plan.poses[local_goal_index].pose.position.x;
    double current_local_goal_y = global_plan.poses[local_goal_index].pose.position.y;
    while (reached_goal(local_goal_index, current_pose))
    {
        local_goal_index++;
        if (local_goal_index >= global_plan.poses.size())
        {
            local_goal_index = global_plan.poses.size() - 1;
            break;
        }
        current_local_goal_x = global_plan.poses[local_goal_index].pose.position.x;
        current_local_goal_y = global_plan.poses[local_goal_index].pose.position.y;
    }

    geometry_msgs::PoseStamped local_goal;
    local_goal.header.frame_id = "map";
    local_goal.header.stamp = ros::Time::now();
    local_goal.pose.position.x = current_local_goal_x;
    local_goal.pose.position.y = current_local_goal_y;
    local_goal.pose.position.z = 0;
    local_goal.pose.orientation = global_plan.poses[local_goal_index].pose.orientation;

    try
    {
        tf_buffer_.transform(local_goal, local_goal, local_goal_frame_id_, ros::Duration(0.1));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }
    return local_goal;
}

bool LocalGoalCreator::reached_goal(int index, geometry_msgs::PoseStamped current_pose)
{
    // ROS_INFO("------------------------------------");
    // ROS_INFO("reached_goal");

    // ROS_INFO("goal_node_id: %d", goal_node_id);
    // ROS_INFO("goal_x: %f goal_y: %f", goal_x, goal_y);
    // ROS_INFO("current_pose_x: %f current_pose_y: %f", current_pose.pose.position.x, current_pose.pose.position.y);
    // ROS_INFO("------------------------------------");

    double goal_x = global_path_.poses[index].pose.position.x;
    double goal_y = global_path_.poses[index].pose.position.y;
    double goal_roll, goal_pitch, goal_yaw;
    tf2::Quaternion q;
    tf2::fromMsg(global_path_.poses[index].pose.orientation, q);
    tf2::Matrix3x3(q).getRPY(goal_roll, goal_pitch, goal_yaw);
    double yaw = atan2(goal_y - current_pose.pose.position.y, goal_x - current_pose.pose.position.x);

    if (pow(current_pose.pose.position.x - goal_x, 2) + pow(current_pose.pose.position.y - goal_y, 2) < pow(local_goal_dist_, 2))
        return true;
    else if (fabs(yaw - goal_yaw) > M_PI / 2.0)
        return true;
    else
        return false;
}

void LocalGoalCreator::process()
{
    ros::Rate rate(hz_);
    while (ros::ok())
    {
        if (global_path_updated_)
        {
            local_goal_index_ = 0;
            global_path_updated_ = false;
            std::cout << "global path updated" << std::endl;
            if (current_pose_updated_)
            {
                double dist_min = 1e6;
                for (int i = 0; i < global_path_.poses.size(); i++)
                {
                    double dist = hypot(global_path_.poses[i].pose.position.x - current_pose_.pose.position.x, global_path_.poses[i].pose.position.y - current_pose_.pose.position.y);
                    if (dist < dist_min)
                    {
                        dist_min = dist;
                        local_goal_index_ = i;
                    }
                }
            }
        }
        if (global_path_.poses.size() == 0)
        {
            std::cout << "global path is empty" << std::endl;
            ros::spinOnce();
            rate.sleep();
            continue;
        }
        if (current_pose_updated_)
        {
            std::cout << "current pose updated" << std::endl;
            geometry_msgs::PoseStamped local_goal = get_local_goal(current_pose_, global_path_, local_goal_index_);
            local_goal_pub_.publish(local_goal);
            current_pose_updated_ = false;
            std::cout << "local goal published" << std::endl;
        }
        ros::spinOnce();
        rate.sleep();
    }
}