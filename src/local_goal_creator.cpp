#include "local_goal_creator/local_goal_creator.h"

LocalGoalCreator::LocalGoalCreator():
    nh_(),
    private_nh_("~")
{
    private_nh_.param("hz", hz_, 10);
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