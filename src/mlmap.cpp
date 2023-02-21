#include "mlmap.h"

void mlmap::init_map(ros::NodeHandle &node_handle)
{
    // awareness_map initialize
    cout << "awareness map initialize" << endl;
    nh = node_handle;
    string configFilePath;
    nh.getParam("/mlmapping_configfile", configFilePath);
    cout << "config file path: " << configFilePath << endl;
    camera2odom_latency = getDoubleVariableFromYaml(configFilePath, "camera2odom_latency");
    // init map
    pc_sample_cnt = getIntVariableFromYaml(configFilePath, "mlmapping_sample_cnt");
    cx_ = getDoubleVariableFromYaml(configFilePath, "mlmapping_cam_cx");
    cy_ = getDoubleVariableFromYaml(configFilePath, "mlmapping_cam_cy");
    fx_ = getDoubleVariableFromYaml(configFilePath, "mlmapping_cam_fx");
    fy_ = getDoubleVariableFromYaml(configFilePath, "mlmapping_cam_fy");
    if_visualize_odds = getBoolVariableFromYaml(configFilePath, "visualize_odds");
    awareness_map = new awareness_map_cylindrical();

    Mat4x4 T_bs_mat = Mat44FromYaml(configFilePath, "T_B_S");
    SE3 T_bs = SE3(T_bs_mat.topLeftCorner(3, 3),
                   T_bs_mat.topRightCorner(3, 1));
    awareness_map->setTbs(T_bs);
    awareness_map->init_map(getDoubleVariableFromYaml(configFilePath, "mlmapping_am_d_Rho"),
                            getDoubleVariableFromYaml(configFilePath, "mlmapping_am_d_Phi_deg"),
                            getDoubleVariableFromYaml(configFilePath, "mlmapping_am_d_Z"),
                            getIntVariableFromYaml(configFilePath, "mlmapping_am_n_Rho"),
                            getIntVariableFromYaml(configFilePath, "mlmapping_am_n_Z_below"),
                            getIntVariableFromYaml(configFilePath, "mlmapping_am_n_Z_over"),
                            getBoolVariableFromYaml(configFilePath, "mlmapping_use_raycasting"),
                            getDoubleVariableFromYaml(configFilePath, "mlmapping_depth_noise_coe"));

    // init transform
    // transformStamped_T_wb
    publish_T_wb = getBoolVariableFromYaml(configFilePath, "publish_T_wb");
    transformStamped_T_wb.header.frame_id = getStringFromYaml(configFilePath, "world_frame_id");
    transformStamped_T_wb.child_frame_id = getStringFromYaml(configFilePath, "body_frame_id");
    transformStamped_T_wb.transform.translation.x = 0;
    transformStamped_T_wb.transform.translation.y = 0;
    transformStamped_T_wb.transform.translation.z = 0;
    transformStamped_T_wb.transform.rotation.x = 0;
    transformStamped_T_wb.transform.rotation.y = 0;
    transformStamped_T_wb.transform.rotation.z = 0;
    transformStamped_T_wb.transform.rotation.w = 1;
    // transformStamped_T_wa
    transformStamped_T_wa = transformStamped_T_wb;
    transformStamped_T_wa.child_frame_id = getStringFromYaml(configFilePath, "awareness_frame_id");
    // transformStamped_T_bs
    publish_T_bs = getBoolVariableFromYaml(configFilePath, "publish_T_bs");
    transformStamped_T_bs.header.frame_id = getStringFromYaml(configFilePath, "body_frame_id");
    transformStamped_T_bs.child_frame_id = getStringFromYaml(configFilePath, "sensor_frame_id");
    transformStamped_T_bs.transform.translation.x = T_bs.translation().x();
    transformStamped_T_bs.transform.translation.y = T_bs.translation().y();
    transformStamped_T_bs.transform.translation.z = T_bs.translation().z();
    transformStamped_T_bs.transform.rotation.x = T_bs.so3().unit_quaternion().x();
    transformStamped_T_bs.transform.rotation.y = T_bs.so3().unit_quaternion().y();
    transformStamped_T_bs.transform.rotation.z = T_bs.so3().unit_quaternion().z();
    transformStamped_T_bs.transform.rotation.w = T_bs.so3().unit_quaternion().w();

    // init publisher
    // awarenessmap_pub = new msg_awareness(nh, "/mlmapping_awareness");
    // a2w_pub = new msg_awareness2local(nh, "/awareness2local", 2);
    // bool use_exactsync = getBoolVariableFromYaml(configFilePath, "use_exactsync");
    // bool use_odom = getBoolVariableFromYaml(configFilePath, "use_odom");

    depth_sub.subscribe(nh, "/mlmapping/depth", 1);
    odom_sub.subscribe(nh, "/mlmapping/odom", 1);
    imu_sub.subscribe(nh, "/mlmapping/imu", 1);

    // local_map initialize
    cout << "local map initialize" << endl;
    local_map = new local_map_cartesian();
    local_map->init_map(getDoubleVariableFromYaml(configFilePath, "mlmapping_subbox_d_xyz"),
                        static_cast<unsigned int>(getIntVariableFromYaml(configFilePath, "mlmapping_subbox_n")),
                        static_cast<float>(getDoubleVariableFromYaml(configFilePath, "mlmapping_lm_log_odds_min")),
                        static_cast<float>(getDoubleVariableFromYaml(configFilePath, "mlmapping_lm_log_odds_max")),
                        static_cast<float>(getDoubleVariableFromYaml(configFilePath, "mlmapping_lm_measurement_hit")),
                        static_cast<float>(getDoubleVariableFromYaml(configFilePath, "mlmapping_lm_measurement_miss")),
                        static_cast<float>(getDoubleVariableFromYaml(configFilePath, "mlmapping_lm_occupied_sh")),
                        getBoolVariableFromYaml(configFilePath, "use_exploration_frontiers"));
    // local_map->allocate_memory_for_local_map();
    cout << "local map initialize finish!" << endl;
    // cout << "frame id: " << getStringFromYaml(configFilePath, "world_frame_id") << endl;
    transformStamped_T_wl.header.frame_id = getStringFromYaml(configFilePath, "world_frame_id");
    transformStamped_T_wl.child_frame_id = getStringFromYaml(configFilePath, "local_frame_id");
    transformStamped_T_wl.header.stamp = ros::Time::now();
    transformStamped_T_wl.transform.translation.x = 0;
    transformStamped_T_wl.transform.translation.y = 0;
    transformStamped_T_wl.transform.translation.z = 0;
    transformStamped_T_wl.transform.rotation.x = 0;
    transformStamped_T_wl.transform.rotation.y = 0;
    transformStamped_T_wl.transform.rotation.z = 0;
    transformStamped_T_wl.transform.rotation.w = 1;

    visulize_raycasting = getBoolVariableFromYaml(configFilePath, "visulize_raycasting");

    // independ modules:
    // enable_project2d = getBoolVariableFromYaml(configFilePath, "use_projected_2d_map");
    // occupancy_grid_publisher = new Local2OccupancyGrid2D(nh, "/occupancy_map", 2);

    // occupancy_grid_publisher->setLocalMap(local_map,
    //                                       getStringFromYaml(configFilePath, "world_frame_id"),
    //                                       getBoolVariableFromYaml(configFilePath, "use_relative_height"),
    //                                       getDoubleVariableFromYaml(configFilePath, "projected_2d_map_max_z"),
    //                                       getDoubleVariableFromYaml(configFilePath, "projected_2d_map_min_z"));
    // enable_esfds = getBoolVariableFromYaml(configFilePath, "use_esdfs");
    // esdfs_publisher = new Local2ESDFsBatch(nh, "/esdfs_aaa", 2);
    // esdfs_publisher->setLocalMap(local_map,
    //                              getStringFromYaml(configFilePath, "awareness_frame_id"),
    //                              getIntVariableFromYaml(configFilePath, "esdfs_n_xy"),
    //                              getIntVariableFromYaml(configFilePath, "esdfs_n_z"),
    //                              getIntVariableFromYaml(configFilePath, "batch_max_search_range"));

    // timer
    // timer_ = nh.createTimer(ros::Duration(0.1), &mlmap::timerCb, this);

    // publisher
    // localmap_pub = new msg_localmap();
    // localmap_pub = new msg_localmap(nh, "/mlmapping_local");

    globalmap_publisher = new rviz_vis();
    globalmap_publisher->set_as_global_map_publisher(nh, "/global_map",
                                                     getStringFromYaml(configFilePath, "world_frame_id"),
                                                     5);
    frontier_publisher = new rviz_vis();
    frontier_publisher->set_as_frontier_publisher(nh, "/frontier",
                                                  getStringFromYaml(configFilePath, "world_frame_id"),
                                                  5);
    odds_publisher = new rviz_vis();
    odds_publisher->set_as_odds_publisher(nh, "/odds",
                                          getStringFromYaml(configFilePath, "world_frame_id"),
                                          5);
    world_id = getStringFromYaml(configFilePath, "world_frame_id");

    map_pub = nh.advertise<visualization_msgs::Marker>("/raycasting", 3);
    odds_pub = nh.advertise<visualization_msgs::MarkerArray>("/odds", 3);
    odds_min = logit_inv(local_map->log_odds_min);
    odds_max = logit_inv(local_map->log_odds_max);

    OdomapproxSync_ = new message_filters::Synchronizer<ApproxSyncPolicyOdom>(ApproxSyncPolicyOdom(100), depth_sub, odom_sub, imu_sub);
    OdomapproxSync_->registerCallback(boost::bind(&mlmap::depth_odom_input_callback, this, _1, _2, _3));
    this->timer1_ = nh.createTimer(ros::Duration(0.1), std::bind(&mlmap::timerCb, this));
    cout << "ApproxSyncPolicy message filter settled!" << endl;
}

void mlmap::timerCb1()
{
    ROS_INFO("Callback 1 triggered");
}

Vec3 mlmap::getcolor(double ratio)
{
    // we want to normalize ratio so that it fits in to 6 regions
    // where each region is 256 units long
    int normalized = int(ratio * 256 * 3);
    // find the distance to the start of the closest region
    int x = normalized % 256;
    int red = 0, grn = 0, blu = 0;
    switch (normalized / 256)
    {
    case 0:
        red = 255;
        grn = x;
        blu = 0;
        break; // red->yellow
    case 1:
        red = 255 - x;
        grn = 255;
        blu = 0;
        break; // yellow->green
    case 2:
        red = 0;
        grn = 255;
        blu = x;
        break; // green->cyan
    case 3:
        red = 0;
        grn = 255 - x;
        blu = 255;
        break; // cyan->blue
    default:
        cout << "ratio: " << ratio << " case: " << normalized / 256 << endl;
        // case 4: red = x;        grn = 0;        blu = 255;     break;//blue
    }

    return Vec3(red / 260.0, grn / 260.0, blu / 260.0);
}

Vec3 mlmap::getcolor_gray(double ratio)
{
    return Vec3(ratio,ratio,ratio);
}

void mlmap::visualize_odds(float height = 0.7)
{
    visualization_msgs::MarkerArray mks;
    visualization_msgs::Marker spheres, lines;
    spheres.header.frame_id = world_id;
    spheres.header.stamp = stamp;
    spheres.ns = "points";
    spheres.type = visualization_msgs::Marker::CUBE_LIST;
    spheres.action = visualization_msgs::Marker::ADD;
    spheres.pose.orientation.w = 1.0;
    spheres.scale.x = spheres.scale.y = 0.2;
    spheres.scale.z = 0.01;
    spheres.id = 0;

    lines.header.frame_id = world_id;
    lines.header.stamp = stamp;
    lines.ns = "lines";
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.action = visualization_msgs::Marker::ADD;
    lines.pose.orientation.w = 1.0;
    lines.scale.x = 0.1;
    lines.id = 0;
    lines.color.r = lines.color.g = lines.color.b = 1.0;
    lines.color.a = 1.0;
    double time_total_getgrad = 0;
    size_t cnt_getgrad = 0;
    for (auto iter = local_map->observed_group_map.begin(); iter != local_map->observed_group_map.end(); iter++)
    {
        int subbox_id = 0;
        // cout<<"global box id: "<<iter->first.transpose()<<" number: "<<cnt++<<endl;
        for (auto it = iter->second.log_odds.begin(); it != iter->second.log_odds.end(); it++)
        {
            // PointP p1 = localmap->subbox_id2xyz_glb(pt,2);
            Vec3 pt = local_map->subbox_id2xyz_glb_vec(iter->first, subbox_id);
            if (pt[2] < height + 1e-3 && pt[2] > height - 1e-3)
            {
                geometry_msgs::Point point;
                point.x = pt[0];
                point.y = pt[1];
                point.z = pt[2];

                spheres.points.push_back(point);
                std_msgs::ColorRGBA color;

                double ratio = logit_inv(*it);
                if (!(0 < ratio && ratio < 1))
                    cout << "this odds: " << *it << endl;
                Vec3 rgb = this->getcolor(ratio);
                color.r = static_cast<float>(rgb(0));
                color.g = static_cast<float>(rgb(1));
                color.b = static_cast<float>(rgb(2));
                color.a = static_cast<float>(0.9);
                spheres.colors.push_back(color);

                if (ratio > 0.8)
                {
                    geometry_msgs::Point point1;
                    auto t1 = std::chrono::system_clock::now();

                    Vec3 grad = getOddGrad(pt);
                    auto t2 = std::chrono::system_clock::now();
                    std::chrono::duration<double> diff = t2 - t1;
                    cnt_getgrad++;
                    time_total_getgrad += diff.count() * 1000;
                    point1.x = point.x + grad[0];
                    point1.y = point.y + grad[1];
                    point1.z = point.z + grad[2];
                    lines.points.emplace_back(point);
                    lines.points.emplace_back(point1);
                }

                // cout<<"point: "<<pc->points.back()<<"glb id: "<<iter->first<<"subbox id: "<<*it<<endl;
                // subbox_id++;
            }
            subbox_id++;
        }
    }
    mks.markers.push_back(spheres);
    mks.markers.push_back(lines);
    cout << "size: " << spheres.points.size() << " average getGrad() time cost(ms): " << time_total_getgrad / cnt_getgrad << endl;
    if (spheres.points.size() != 0)
    {
        this->odds_pub.publish(mks);
    }
}
void mlmap::project_depth()
{
    uint16_t *row_ptr;
    // int cols = current_img_.cols, rows = current_img_.rows;
    int cols = depth_image_.cols;
    int rows = depth_image_.rows;
    size_t u, v;
    double depth;
    Vec3 pt_cur;
    int cnt = 0;
    int max_iter = 2 * pc_sample_cnt;
    while (pc_eigen.size() < pc_sample_cnt && cnt < max_iter)
    {
        cnt++;
        v = static_cast<size_t>(rand() % rows);
        u = static_cast<size_t>(rand() % cols);
        row_ptr = depth_image_.ptr<uint16_t>(v) + u;
        // row_ptr += u;
        depth = (*row_ptr) * inv_factor;

        // filter depth
        // depth += rand_noise_(eng_);
        // if (depth > 0.01) depth += rand_noise2_(eng_);
        // if (depth > 7)
        // {
        //     cout<<"depth > 7: "<<depth<<" u v: "<<u<<" "<<v<<endl;
        // }
        if (*row_ptr == 0)
        {
            continue;
        }

        // project to world frame
        pt_cur(0) = (u - cx_) * depth / fx_;
        pt_cur(1) = (v - cy_) * depth / fy_;
        pt_cur(2) = depth;
        pc_eigen.emplace_back(pt_cur);
    }
}

void mlmap::visualize_raycast()
{

    visualization_msgs::Marker spheres;
    spheres.header.frame_id = "awareness_frame";
    spheres.header.stamp = stamp;
    spheres.ns = "points";
    spheres.type = visualization_msgs::Marker::SPHERE_LIST;
    spheres.action = visualization_msgs::Marker::ADD;
    spheres.pose.orientation.w = 1.0;
    spheres.scale.x = spheres.scale.y = spheres.scale.z = 0.1;
    spheres.id = 0;
    for (auto idx : awareness_map->miss_idx_set)
    {
        geometry_msgs::Point point;
        point.x = awareness_map->map->at(idx).center_pt[0];
        point.y = awareness_map->map->at(idx).center_pt[1];
        point.z = awareness_map->map->at(idx).center_pt[2];
        spheres.points.push_back(point);
        std_msgs::ColorRGBA color;
        color.r = static_cast<float>(1.0);
        color.g = static_cast<float>(0.0);
        color.b = static_cast<float>(0.0);
        color.a = static_cast<float>(1.0);
        spheres.colors.push_back(color);
    }
    if (spheres.points.size() != 0)
    {
        this->map_pub.publish(spheres);
    }
}
void mlmap::update_map()
{
    awareness_map->input_pc_pose(pc_eigen, T_wb);
    local_map->input_pc_pose_direct(awareness_map);
}

void mlmap::setFree_map_in_bound(Vec3 box_min, Vec3 box_max)
{
    Vec3I glb_id;
    size_t subbox_id;
    for (double x = box_min[0]; x <= box_max[0]; x += local_map->map_dxyz_obv_sub)
    {
        for (double y = box_min[1]; y <= box_max[1]; y += local_map->map_dxyz_obv_sub)
        {
            for (double z = box_min[2]; z <= box_max[2]; z += local_map->map_dxyz_obv_sub)
            {
                local_map->get_global_idx(Vec3(x, y, z), glb_id, subbox_id);
                if (local_map->observed_group_map.find(glb_id) != local_map->observed_group_map.end() && local_map->observed_group_map[glb_id].occupancy.size() > 1)
                {
                    local_map->observed_group_map[glb_id].occupancy[subbox_id] = 'f';
                    local_map->observed_group_map[glb_id].log_odds[subbox_id] = 0;
                }
            }
        }
    }
}

void mlmap::visualize_map()
{
    globalmap_publisher->pub_global_local_map(local_map, stamp);
}

void mlmap::visualize_frontier()
{
    frontier_publisher->pub_frontier(local_map, stamp);
}

void mlmap::timerCb()
{
    // cout << "publish tfs!" << endl;
    SE3 T_wl = local_map->T_wl;
    transformStamped_T_wl.transform.translation.x = T_wl.translation().x();
    transformStamped_T_wl.transform.translation.y = T_wl.translation().y();
    transformStamped_T_wl.transform.translation.z = T_wl.translation().z();
    transformStamped_T_wl.header.stamp = stamp;
    br.sendTransform(transformStamped_T_wl);

    if (publish_T_wb)
    {
        transformStamped_T_wb.header.stamp = stamp;
        transformStamped_T_wb.transform.translation.x = T_wb.translation().x();
        transformStamped_T_wb.transform.translation.y = T_wb.translation().y();
        transformStamped_T_wb.transform.translation.z = T_wb.translation().z();
        transformStamped_T_wb.transform.rotation.x = T_wb.so3().unit_quaternion().x();
        transformStamped_T_wb.transform.rotation.y = T_wb.so3().unit_quaternion().y();
        transformStamped_T_wb.transform.rotation.z = T_wb.so3().unit_quaternion().z();
        transformStamped_T_wb.transform.rotation.w = T_wb.so3().unit_quaternion().w();
        br.sendTransform(transformStamped_T_wb);
        transformStamped_T_wa.header.stamp = stamp;
        transformStamped_T_wa.transform.translation = transformStamped_T_wb.transform.translation;
        br.sendTransform(transformStamped_T_wa);
    }
    if (publish_T_bs)
    {
        transformStamped_T_bs.header.stamp = stamp;
        br.sendTransform(transformStamped_T_bs);
    }

    if (enable_project2d)
    {
        occupancy_grid_publisher->pub_occupancy_grid_2D_from_localmap(local_map, stamp);
        // sum_t_2d += tt.dT_ms();
    }
    if (enable_esfds)
    {
        // tic_toc_ros tt;
        esdfs_publisher->pub_ESDF_3D_from_localmap(local_map, stamp);
        // sum_t_esfd += tt.dT_ms();
    }
}

void mlmap::depth_odom_input_callback(const sensor_msgs::Image::ConstPtr &img_Ptr,
                                      const nav_msgs::Odometry::ConstPtr &pose_Ptr,
                                      const sensor_msgs::Imu::ConstPtr &imu_Ptr)
{
    // cout<<"mlmap got data!"<<endl;
    has_data = true;
    pc_eigen.clear();
    double ros_time_gap_odom = (img_Ptr->header.stamp - pose_Ptr->header.stamp).toSec();
    double ros_time_gap_imu = (img_Ptr->header.stamp - imu_Ptr->header.stamp).toSec();
    // cout << "Time gap between pcl and odom (ms): " << ros_time_gap_odom * 1000 << " pcl and imu (ms): " << ros_time_gap_imu * 1000 << endl;
    double time_gap = ros_time_gap_imu - camera2odom_latency; // pcl time - odom time
    auto t1 = std::chrono::system_clock::now();

    /* get depth image */
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_Ptr, img_Ptr->encoding);

    if (img_Ptr->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
    }
    cv_ptr->image.copyTo(depth_image_);

    SO3 rot_og = SO3(Quaterniond(pose_Ptr->pose.pose.orientation.w,
                                 pose_Ptr->pose.pose.orientation.x,
                                 pose_Ptr->pose.pose.orientation.y,
                                 pose_Ptr->pose.pose.orientation.z));
    Vec3 rot_dot = rot_og.matrix() * Vec3(imu_Ptr->angular_velocity.x, imu_Ptr->angular_velocity.y, imu_Ptr->angular_velocity.z);
    Vec3 rot_cp = rot_og.log() + time_gap * rot_dot; // lie algebra
    T_wb = SE3(SO3::exp(rot_cp),                     // lie group
               Vec3(pose_Ptr->pose.pose.position.x,
                    pose_Ptr->pose.pose.position.y,
                    pose_Ptr->pose.pose.position.z) +
                   (ros_time_gap_odom - camera2odom_latency) * Vec3(pose_Ptr->twist.twist.linear.x,
                                                                    pose_Ptr->twist.twist.linear.y,
                                                                    pose_Ptr->twist.twist.linear.z));
    // compensate the pose gap during the time_gap via linear model. THe position
    //  and orientation are forwarded at the timestamp of point cloud
    stamp = img_Ptr->header.stamp;

    // auto t10 = std::chrono::system_clock::now();
    project_depth();
    // auto t11 = std::chrono::system_clock::now();

    update_map(); 
    map_updated = true;

    // main map update procedure is completed
    auto t2 = std::chrono::system_clock::now();
    visualize_map();
    if (local_map->apply_explored_area)
    visualize_frontier();
    if (if_visualize_odds)
    visualize_odds(0.9);
    // odds_publisher->pub_odd_slice(local_map, stamp, std::bind(&mlmap::getOddGrad, this, _1, 5));
    // timerCb();

    std::chrono::duration<double> diff = t2 - t1;
    // std::chrono::duration<double> diff = t11 - t10;
    time_total += diff.count() * 1000;
    if (count % 10 ==0)
    printf("[mlmapping] map single time: %.4f ms, ave-time cost: %.4f ms\n", diff.count() * 1000, time_total / (++count));

    //        sum_t+=tt.dT_ms();
    //        count++;
    //    cout << awareness_map->l2g_msg_hit_pts_l.size()<<" "<<awareness_map->l2g_msg_miss_pts_l.size() << endl;
}