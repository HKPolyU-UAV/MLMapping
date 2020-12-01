#ifndef RVIZ_VIS_H
#define RVIZ_VIS_H
#include <ros/ros.h>
#include <map_awareness.h>
#include <map_local.h>
#include <map_warehouse.h>
#include <utils/include/all_utils.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>

class rviz_vis
{
public:
    rviz_vis();
    ~rviz_vis();

    ros::Publisher map_pub;
    string frame_id;
    double min_z;
    double max_z;
    double range_z;
    double cube_size_xyz;

    //for localmap
    void set_as_awareness_map_publisher(ros::NodeHandle& nh,
                                        string topic_name,
                                        string frame_id,
                                        unsigned int buffer_size,
                                        awareness_map_cylindrical* awareness_map);

    //for locallmap
    void set_as_local_map_publisher(ros::NodeHandle& nh,
                                    string topic_name,
                                    string frame_id,
                                    unsigned int buffer_size,
                                    local_map_cartesian* localmap);

    //for globalmap
    void set_as_global_map_publisher(ros::NodeHandle& nh,
                                     string topic_name,
                                     string frame_id,
                                     unsigned int buffer_size);

    void pub_global_map(map_warehouse* warehouse,
                        const ros::Time stamp);


    void pub_awareness_map(awareness_map_cylindrical* localmap,
                           const ros::Time stamp);

    void pub_local_map(local_map_cartesian* localmap,
                       const ros::Time stamp);

    void pub_global_local_map(map_warehouse* warehouse,
                              local_map_cartesian* localmap,
                              const ros::Time stamp);

};

#endif // RVIZ_VIS_H
