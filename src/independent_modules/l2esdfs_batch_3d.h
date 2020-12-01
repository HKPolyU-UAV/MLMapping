#ifndef GLOBAL2ESDF3D_H
#define GLOBAL2ESDF3D_H

#include <nav_msgs/OccupancyGrid.h>
#include <map_local.h>
#include <utils/include/all_utils.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

typedef Matrix<double, Dynamic, Dynamic> MatrixXd;
typedef Matrix<bool,   Dynamic, Dynamic> MatrixXb;

class Local2ESDFsBatch
{
  int map3d_nx;
  int map3d_ny;
  int map3d_nz;
  int batch_size_nxy;
  int batch_size_nz;
  int map2mat_bias_x;
  int map2mat_bias_y;
  double mat2vismap_bias_x;
  double mat2vismap_bias_y;
  double map3d_dx;
  double map3d_dy;
  double map3d_dz;
  double esdf_cutoff_value;
  string awareness_frame_id;
  Vec3 visualize_min_xyz;
  int visualized_layer;
  vector<MatrixXd> esdf_map3d;

  visualization_msgs::Marker cubes_array;

  double esdfs_batch_occupied;
  int batch_max_search_range;

  Vec3 esdf_cube_coler(double ratio);
  Vec3I esdf_cube_coler_int(double ratio);
public:
  ros::Publisher esdf_pub;
  ros::Publisher esdf_pc_pub;
  Local2ESDFsBatch(ros::NodeHandle& nh, string topic_name, int buffersize);
  void setLocalMap(local_map_cartesian *map,
                   string awareness_frame_name,
                   int batch_size_nxy_in,
                   int batch_size_nz_in,
                   int batch_max_search_range_in,
                   double esdfs_batch_occupied_in=0.0);
  void pub_ESDF_3D_from_localmap(local_map_cartesian *map, ros::Time stamp);
};

#endif // GLOBAL2ESFD_H
