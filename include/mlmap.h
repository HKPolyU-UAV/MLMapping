#ifndef MLMAP_H
#define MLMAP_H

#include <map_local.h>
#include <map_awareness.h>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <all_utils.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rviz_vis.h>
// #include <msg_awareness2local.h>
// #include <msg_awareness.h>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/TransformStamped.h>

#include <pcl/filters/voxel_grid.h>
// #include <msg_localmap.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <independent_modules/l2grid2d.h>
#include <independent_modules/l2esdfs_batch_3d.h>
#include <chrono>
#include <numeric>
#include <iostream>

#define logit(x) (log10((x) / (1 - (x))))
#define logit_inv(x) (pow(10, x) / (1 + pow(10, x)))

class mlmap
{
private:
    typedef Vec3 (*Fun_odds)(Vec3, size_t);
    // original in awareness map node:
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry, sensor_msgs::Imu> ApproxSyncPolicyOdom;
    message_filters::Synchronizer<ApproxSyncPolicyOdom> *OdomapproxSync_;
    // publisher
    // msg_awareness2local *a2w_pub;
    // msg_awareness *awarenessmap_pub;
    tf2_ros::TransformBroadcaster br;
    // Timer
    ros::Timer inflate_timer_, tf_timer_;
    int count = 0;
    double time_total = 0;
    double camera2odom_latency = 0;
    SE3 T_wb;
    ros::Time stamp;
    size_t pc_sample_cnt;
    bool publish_T_wb;
    bool publish_T_bs;
    bool if_visualize_odds;
    geometry_msgs::TransformStamped transformStamped_T_wb;
    geometry_msgs::TransformStamped transformStamped_T_wa;
    geometry_msgs::TransformStamped transformStamped_T_bs;
    // original in local map node:

    // msg_localmap *localmap_pub;
    rviz_vis *globalmap_publisher;
    rviz_vis *frontier_publisher;
    rviz_vis *odds_publisher;
    geometry_msgs::TransformStamped transformStamped_T_wl;
    bool visulize_raycasting;
    bool enable_project2d;
    bool enable_esfds;
    Local2OccupancyGrid2D *occupancy_grid_publisher;
    Local2ESDFsBatch *esdfs_publisher;
    float total_t;
    // max_ray_length_ =
    const double k_depth_scaling_factor_ = 1000.0;
    const double inv_factor = 1.0 / k_depth_scaling_factor_;
    int skip_pixel_;
    ros::Publisher map_pub, odds_pub;
    ros::NodeHandle nh;
    vector<Vec3> pc_eigen; // body frame
    cv::Mat depth_image_;
    float cx_, cy_, fx_, fy_;
    float odds_min, odds_max;
    string world_id;
    int inflate_global_n = 2;
    vector<Vec3I> glb_idx_nb_list = vector<Vec3I>(6);
    vector<size_t> subbox_id_nb_list = vector<size_t>(6);
    Vec3 ct_pos;
    void tf_timerCb();
    void inflate_timerCb();
    void depth_odom_input_callback(const sensor_msgs::Image::ConstPtr &img_Ptr,
                                   const nav_msgs::Odometry::ConstPtr &pose_Ptr,
                                   const sensor_msgs::Imu::ConstPtr &imu_Ptr);

public:
    typedef std::shared_ptr<mlmap> Ptr;
    awareness_map_cylindrical *awareness_map;
    local_map_cartesian *local_map;
    enum
    {
        FREE = 1,
        OCCUPIED = 0,
        UNKNOWN = -1
    };
    bool has_data = false;
    bool map_updated = false;
    void init_map(ros::NodeHandle &node_handle);

    void project_depth();
    void update_map();

    void setFree_map_in_bound(Vec3 box_min, Vec3 box_max);

    inline int getOccupancy(const Vec3 &pos_w);
    inline int getOccupancy(const Vec3 &pos_w, float inflate);
    inline int getInflateOccupancy(const Vec3 &pos_w);
    inline float getOdd(const Vec3 &pos_w);
    inline float getOdd(const Vec3I &glb_id, size_t subbox_id);

    inline Vec3 getOddGrad(const Vec3 &pos_w, const size_t max_iter);
    // inline Vec3 getOddGrad(Vec3I glb_id, size_t subbox_id, size_t max_iter);
    void visualize_map();

    void visualize_frontier();
    void visualize_raycast();
    void visualize_odds(float height);
    Vec3 getcolor(double ratio);
    Vec3 getcolor_gray(double ratio);
    void inflate_map();
};

inline int mlmap::getOccupancy(const Vec3 &pos_w, float inflate)
{
    if (getOccupancy(pos_w) != OCCUPIED &&
        getOccupancy(pos_w + Vec3(0, 0, inflate)) != OCCUPIED &&
        getOccupancy(pos_w + Vec3(0, 0, -inflate)) != OCCUPIED &&
        getOccupancy(pos_w + Vec3(0, inflate, 0)) != OCCUPIED &&
        getOccupancy(pos_w + Vec3(0, -inflate, 0)) != OCCUPIED &&
        getOccupancy(pos_w + Vec3(inflate, 0, 0)) != OCCUPIED &&
        getOccupancy(pos_w + Vec3(-inflate, 0, 0)) != OCCUPIED && // x y z axis

        getOccupancy(pos_w + Vec3(-inflate, inflate, 0)) != OCCUPIED &&
        getOccupancy(pos_w + Vec3(-inflate, -inflate, 0)) != OCCUPIED &&
        getOccupancy(pos_w + Vec3(inflate, inflate, 0)) != OCCUPIED &&
        getOccupancy(pos_w + Vec3(inflate, -inflate, 0)) != OCCUPIED && // xy plane

        getOccupancy(pos_w + Vec3(0, -inflate, inflate)) != OCCUPIED &&
        getOccupancy(pos_w + Vec3(0, -inflate, -inflate)) != OCCUPIED &&
        getOccupancy(pos_w + Vec3(0, inflate, inflate)) != OCCUPIED &&
        getOccupancy(pos_w + Vec3(0, inflate, -inflate)) != OCCUPIED && // yz plane

        getOccupancy(pos_w + Vec3(-inflate, 0, inflate)) != OCCUPIED &&
        getOccupancy(pos_w + Vec3(-inflate, 0, -inflate)) != OCCUPIED &&
        getOccupancy(pos_w + Vec3(inflate, 0, inflate)) != OCCUPIED &&
        getOccupancy(pos_w + Vec3(inflate, 0, -inflate)) != OCCUPIED) // xz plane
        return FREE;
    else
        return OCCUPIED;
}
inline int mlmap::getOccupancy(const Vec3 &pos_w)
{
    Vec3I glb_id;
    size_t subbox_id;
    char res;
    // Vec3I xyz_idx_local;
    // if (local_map->xyz2xyzIdxwithBoderCheck(pos_w, xyz_idx_local))
    // {
    //     return local_map->map->at(local_map->mapIdx(xyz_idx)).is_occupied;
    // }
    local_map->get_global_idx(pos_w, glb_id, subbox_id);
    if (local_map->observed_group_map.find(glb_id) == local_map->observed_group_map.end())
        return UNKNOWN;
    else if (local_map->observed_group_map[glb_id].occupancy.size() == 1)
        res = local_map->observed_group_map[glb_id].occupancy[0];
    else
        res = local_map->observed_group_map[glb_id].occupancy[subbox_id];
    if (res == 'o')
        return OCCUPIED;
    else if (res == 'f')
        return FREE;
    else
        return UNKNOWN;
}

inline int mlmap::getInflateOccupancy(const Vec3 &pos_w)
{
    Vec3I glb_id;
    size_t subbox_id;
    local_map->get_global_idx(pos_w, glb_id, subbox_id);
    if (local_map->observed_group_map.find(glb_id) == local_map->observed_group_map.end())
        return UNKNOWN;
    else if (local_map->observed_group_map[glb_id].occupancy.size() == 1)
        return UNKNOWN;
    else
    {
    if (local_map->observed_group_map[glb_id].inflate_occupancy[subbox_id] == 'o')
        return OCCUPIED;
    else
        return UNKNOWN;
    }
}

inline float mlmap::getOdd(const Vec3 &pos_w) // return true odd
{

    Vec3I glb_id;
    size_t subbox_id;
    local_map->get_global_idx(pos_w, glb_id, subbox_id);
    if (local_map->observed_group_map.find(glb_id) == local_map->observed_group_map.end())
        return 0.5;
    else if (local_map->observed_group_map[glb_id].log_odds.size() == 1)
        return logit_inv(local_map->observed_group_map[glb_id].log_odds[0]);
    else
        return logit_inv(local_map->observed_group_map[glb_id].log_odds[subbox_id]);
}

inline float mlmap::getOdd(const Vec3I &glb_id, size_t subbox_id)
{
    if (local_map->observed_group_map.find(glb_id) == local_map->observed_group_map.end())
        return 0.5;
    else if (local_map->observed_group_map[glb_id].log_odds.size() == 1)
        return logit_inv(local_map->observed_group_map[glb_id].log_odds[0]);
    else
        return logit_inv(local_map->observed_group_map[glb_id].log_odds[subbox_id]);
}

inline Vec3 mlmap::getOddGrad(const Vec3 &pos_w, size_t max_iter = 5)
{
    Vec3I glb_id;
    size_t subbox_id;
    local_map->get_global_idx(pos_w, glb_id, subbox_id);
    Vec3I glb_idx_nb, glb_idx_nb_min;
    size_t subbox_id_nb, subbox_id_nb_min;
    float min_odd = getOdd(glb_id, subbox_id);
    float ori_odd = min_odd;
    float tmp_odd;
    bool flag = false;
    // Vec3 Grad_drt;
    // Mat6x4I neighbors = local_map->subbox_neighbors[subbox_id];
    size_t iter;
    // cout<<"Neighbor Mat:\n"<<neighbors<<endl;
    for (iter = 0; iter < max_iter && !flag; iter++)
    {
        for (auto i = 0; i < 6; i++)
        {
            if (iter == 0)
            {
                glb_idx_nb = glb_id + local_map->subbox_neighbors[subbox_id].block(i, 0, 1, 3).transpose(); // add the displacement of global idx
                subbox_id_nb = local_map->subbox_neighbors[subbox_id](i, 3);

                // cout << "glb_id: " << glb_id.transpose() << " subbox_id: " <<subbox_id
                //      << " glb_idx_nb: " << glb_idx_nb.transpose()<<" subbox_id_nb: " <<subbox_id_nb<<
                //      " glb disp: "<<local_map->subbox_neighbors[subbox_id].block(i, 0, 1, 3)<< endl;
            }
            else
            {
                glb_idx_nb = glb_idx_nb_list[i] + local_map->subbox_neighbors[subbox_id_nb_list[i]].block(i, 0, 1, 3).transpose();
                subbox_id_nb = local_map->subbox_neighbors[subbox_id_nb_list[i]](i, 3); // keep searching along the original direction
            }
            glb_idx_nb_list[i] = glb_idx_nb;
            subbox_id_nb_list[i] = subbox_id_nb;
            tmp_odd = getOdd(glb_idx_nb, subbox_id_nb);
            if (tmp_odd < min_odd)
            {
                min_odd = tmp_odd;
                glb_idx_nb_min = glb_idx_nb;
                subbox_id_nb_min = subbox_id_nb;
                flag = true;
            }
        }
    }
    if (flag)
    {
        // cout << "pos_w: " << pos_w.transpose() << " grad ori_dist: " << (local_map->subbox_id2xyz_glb_vec(glb_idx_nb_min, subbox_id_nb_min) - pos_w).transpose()
        //  << " odd diff: " << (ori_odd - min_odd) << endl;

        return (local_map->subbox_id2xyz_glb_vec(glb_idx_nb_min, subbox_id_nb_min) - pos_w) * (ori_odd - min_odd);
    }
    else
    {
        // cout <<"iter: "<<iter<< " pos_w: " << pos_w.transpose() << " grad ori_dist: " << (local_map->subbox_id2xyz_glb_vec(glb_idx_nb_min, subbox_id_nb_min) - pos_w).transpose()
        //  << " odd diff: " << (ori_odd - min_odd) << endl;
        return Vec3(0.0, 0.0, 0.0);
    }
}

// inline Vec3 mlmap::getOddGrad(Vec3I glb_id, size_t subbox_id, size_t max_iter = 5)
// {
//     // Vec3I glb_id;
//     // size_t subbox_id;
//     // local_map->get_global_idx(pos_w, glb_id, subbox_id);
//     Vec3I glb_idx_nb, glb_idx_nb_min;
//     size_t subbox_id_nb, subbox_id_nb_min;
//     float min_odd = getOdd(glb_id, subbox_id);
//     float ori_odd = min_odd;
//     float tmp_odd;
//     bool flag = false;
//     Vec3 Grad_drt;
//     // Mat6x4I neighbors = local_map->subbox_neighbors[subbox_id];
//     for (size_t iter = 0; iter < max_iter && !flag; iter++)
//     {
//         for (auto i = 0; i < 6; i++)
//         {
//             if (iter == 0)
//             {
//                 glb_idx_nb = glb_id + local_map->subbox_neighbors[subbox_id].block(i, 0, 1, 3); // add the displacement of global idx
//                 subbox_id_nb = local_map->subbox_neighbors[subbox_id](i, 3);
//             }
//             else
//             {
//                 glb_idx_nb = glb_idx_nb_list[i] + local_map->subbox_neighbors[subbox_id_nb_list[i]].block(i, 0, 1, 3);
//                 subbox_id_nb = local_map->subbox_neighbors[subbox_id_nb_list[i]](i, 3); // keep searching along the original direction
//             }
//             glb_idx_nb_list[i] = glb_idx_nb;
//             subbox_id_nb_list[i] = subbox_id_nb;
//             tmp_odd = getOdd(glb_idx_nb, subbox_id_nb);
//             if (tmp_odd < min_odd)
//             {
//                 min_odd = tmp_odd;
//                 glb_idx_nb_min = glb_idx_nb;
//                 subbox_id_nb_min = subbox_id_nb;
//                 flag = true;
//             }
//         }
//     }
//     if (flag)
//         return (local_map->subbox_id2xyz_glb_vec(glb_idx_nb_min, subbox_id_nb_min) - pos_w) * (ori_odd - min_odd) * local_map->map_reso_inv;
//     else
//         return Vec3(0.0, 0.0, 0.0);
// }
#endif