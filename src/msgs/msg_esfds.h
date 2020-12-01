#ifndef msg_awareness2local_H
#define msg_awareness2local_H

#include <mlmapping/local2global.h>
#include <utils/include/all_utils.h>
#include <ros/ros.h>
#include <ros/publisher.h>


class msg_awareness2local
{
public:
    ros::Publisher l2g_pub;

    msg_awareness2local();
    msg_awareness2local(ros::NodeHandle& nh, string topic_name, int buffersize=2);
    void pub(const SE3 T_w_l, vector<Vec3> obs_pts, vector<Vec3> miss_pts, ros::Time stamp=ros::Time::now());
    static void unpack(mlmapping::local2globalConstPtr ptr,
                       SE3             &T_w_l,
                       vector<Vec3>    &l2g_obs_l,
                       vector<Vec3>    &l2g_miss_l,
                       ros::Time       &T);
};

#endif // msg_awareness2local_H
