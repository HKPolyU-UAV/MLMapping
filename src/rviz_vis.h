#ifndef RVIZ_VIS_H
#define RVIZ_VIS_H
#include <ros/ros.h>
#include <map_awareness.h>
#include <map_local.h>
// #include <mlmap.h>
#include <utils/include/all_utils.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#define logit_inv(x) (pow(10, x) / (1 + pow(10, x)))
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
    typedef Vec3(*Fun_odds)(Vec3,size_t);
    // for localmap
    void set_as_awareness_map_publisher(ros::NodeHandle &nh,
                                        string topic_name,
                                        string frame_id,
                                        unsigned int buffer_size,
                                        awareness_map_cylindrical *awareness_map);

    // for locallmap
    void set_as_local_map_publisher(ros::NodeHandle &nh,
                                    string topic_name,
                                    string frame_id,
                                    unsigned int buffer_size,
                                    local_map_cartesian *localmap);

    // for globalmap
    void set_as_global_map_publisher(ros::NodeHandle &nh,
                                     string topic_name,
                                     string frame_id,
                                     unsigned int buffer_size);

    void set_as_frontier_publisher(ros::NodeHandle &nh,
                                   string topic_name,
                                   string frame_id,
                                   unsigned int buffer_size);

    void set_as_odds_publisher(ros::NodeHandle &nh,
                               string topic_name,
                               string frame_id,
                               unsigned int buffer_size);

    void pub_awareness_map(awareness_map_cylindrical *localmap,
                           const ros::Time stamp);

    void pub_local_map(local_map_cartesian *localmap,
                       const ros::Time stamp);

    void pub_global_local_map(
        local_map_cartesian *localmap,
        const ros::Time stamp);
    void pub_frontier(local_map_cartesian *localmap,
                      const ros::Time stamp);
    void pub_odd_slice(local_map_cartesian *localmap,
                       const ros::Time stamp, Fun_odds fun);
};

#endif // RVIZ_VIS_H
