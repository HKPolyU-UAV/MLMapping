#include "l2grid2d.h"

Local2OccupancyGrid2D::Local2OccupancyGrid2D(ros::NodeHandle& nh, string topic_name, unsigned int buffersize)
{
    this->occupancygrid_pub = nh.advertise<nav_msgs::OccupancyGrid>(topic_name, buffersize);
}

void Local2OccupancyGrid2D::setLocalMap(local_map_cartesian* map,
                                        string world_frame_id,
                                        bool   relative_height_in,
                                        double max_z_in,
                                        double min_z_in)
{
    map2d_nx = map2d_ny = map->map_nxy;
    map2d_dx = map2d_dy = map->map_dxyz;


    occupancy_grid.header.frame_id = world_frame_id;
    occupancy_grid.data.clear();
    occupancy_grid.info.resolution = map2d_dx;         // float32
    occupancy_grid.info.width      = map2d_nx;           // uint32
    occupancy_grid.info.height     = map2d_ny;           // uint32

//    Vec3 offset = map->map_min_xyz - map->map_center_xyz;
//    cout << "offset:" << offset.transpose() << endl;
//    occupancy_grid.info.origin.position.x = - (0.5*map->map_dxyz)-floor(map->map_nxy/2.0)*map->map_dxyz;;
//    occupancy_grid.info.origin.position.y = - (0.5*map->map_dxyz)-floor(map->map_nxy/2.0)*map->map_dxyz;
//    occupancy_grid.info.origin.position.z = - (0.5*map->map_dxyz)-floor(map->map_nz/2.0)*map->map_dxyz;

    max_z = max_z_in;
    min_z = min_z_in;
    for (unsigned int i = 0; i < map2d_nx*map2d_ny; i++)
    {
        occupancy_grid.data.push_back(0);
    }
}

void Local2OccupancyGrid2D::pub_occupancy_grid_2D_from_localmap(local_map_cartesian *map, ros::Time stamp)
{
    occupancy_grid.header.stamp = stamp;

    occupancy_grid.info.origin.position.x = map->map_min_xyz(0);
    occupancy_grid.info.origin.position.y = map->map_min_xyz(1);
    occupancy_grid.info.origin.position.z = map->map_min_xyz(2);

    for (auto &grid: occupancy_grid.data) {
        grid = 0;
    }
    double sh_z_max;
    double sh_z_min;
    if(use_relative_height)
    {
        sh_z_max = max_z + map->T_wa_latest.translation().z();
        sh_z_min = min_z + map->T_wa_latest.translation().z();
    }else
    {//absolute height
        sh_z_max = max_z;
        sh_z_min = min_z;
    }
    int max_z_idx =  static_cast<int>(round((sh_z_max-map->map_min_z)/map->map_dxyz));
    int min_z_idx =  static_cast<int>(round((sh_z_min-map->map_min_z)/map->map_dxyz));
    for(auto idx : map->occupied_cell_idx)
    {
        int z = static_cast<int>(map->map->at(idx).idx_z);
        if(z<max_z_idx && z>min_z_idx)
        {
            occupancy_grid.data.at(map->map->at(idx).project2d_idx)=100;
        }
    }
    this->occupancygrid_pub.publish(occupancy_grid);
}
