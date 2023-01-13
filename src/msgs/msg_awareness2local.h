#ifndef msg_awareness2local_H
#define msg_awareness2local_H

#include <mlmapping/awareness2local.h>
#include <utils/include/all_utils.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <map_awareness.h>

class msg_awareness2local
{
public:
    ros::Publisher l2g_pub;

    msg_awareness2local();
    msg_awareness2local(ros::NodeHandle& nh, string topic_name, int buffersize=2);
    void pub_a2l(awareness_map_cylindrical *a_map, ros::Time stamp);
    static void unpack(mlmapping::awareness2localConstPtr ptr,
                       SE3             &T_w_a,
                       vector<Vec3>    &l2g_obs_l,
                       vector<float>   &l2g_odds_l,
                       vector<Vec3>    &l2g_miss_l,
                       ros::Time       &T);
};

#endif // msg_awareness2local_H
