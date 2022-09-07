#ifndef __LOCAL_GOAL_CREATOR_H__
#define __LOCAL_GOAL_CREATOR_H__

#include <ros/ros.h>

class LocalGoalCreator
{
    public:
        LocalGoalCreator();
        void process();

    private:
        int hz_;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
};

#endif // __LOCAL_GOAL_CREATOR_H__