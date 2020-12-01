#ifndef GLOBAL2OCCUPANCYGRID2D_H
#define GLOBAL2OCCUPANCYGRID2D_H

#include <nav_msgs/OccupancyGrid.h>
#include <map_local.h>
#include <utils/include/all_utils.h>
#include <ros/ros.h>
#include <ros/publisher.h>

class Local2OccupancyGrid2D
{
    unsigned int map2d_nx;
    unsigned int map2d_ny;
    float map2d_dx;
    float map2d_dy;
    bool use_relative_height;
    double max_z, min_z;
    int max_z_idx;
    int min_z_idx;

    nav_msgs::OccupancyGrid occupancy_grid;


public:
    ros::Publisher occupancygrid_pub;
    
    Local2OccupancyGrid2D(ros::NodeHandle& nh, string topic_name, unsigned int buffersize=2);
    void setLocalMap(local_map_cartesian* map,
                     string world_frame_id,
                     bool relative_height_in,
                     double max_z_in,
                     double min_z_in);
    void pub_occupancy_grid_2D_from_localmap(local_map_cartesian *map, ros::Time stamp=ros::Time::now());

};

#endif // GLOBAL2OCCUPANCYGRID2D_H
