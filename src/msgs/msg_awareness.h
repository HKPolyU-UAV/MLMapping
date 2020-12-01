#ifndef msg_awareness_H
#define msg_awareness_H

#include <mlmapping/awareness.h>
#include <map_awareness.h>
#include <utils/include/all_utils.h>
#include <ros/ros.h>
#include <ros/publisher.h>


class msg_awareness
{
public:
    ros::Publisher awarenessmap_pub;

    msg_awareness();
    msg_awareness(ros::NodeHandle& nh,
                 string topic_name,
                 int buffersize=2);
    void pub(awareness_map_cylindrical *map,
             ros::Time stamp);
    static void unpack(mlmapping::awarenessConstPtr msg_ptr,
                awareness_map_cylindrical *map);
};

#endif // msg_awareness_H
