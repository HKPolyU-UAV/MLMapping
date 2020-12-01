#ifndef msg_localmap_H
#define msg_localmap_H

#include <mlmapping/localmap.h>
#include <map_local.h>
#include <utils/include/all_utils.h>
#include <ros/ros.h>
#include <ros/publisher.h>


class msg_localmap
{
public:
    ros::Publisher localmap_pub;

    msg_localmap();
    msg_localmap(ros::NodeHandle& nh,
                 string topic_name,
                 int buffersize=2);
    void pub(local_map_cartesian *map,
             ros::Time stamp);
    static void unpack(mlmapping::localmapConstPtr msg_ptr,
             local_map_cartesian *map);
};

#endif // msg_awareness2local_H
