#include "msg_awareness2local.h"

msg_awareness2local::msg_awareness2local()
{

}

msg_awareness2local::msg_awareness2local(ros::NodeHandle& nh, string topic_name, int buffersize)
{
    l2g_pub = nh.advertise<mlmapping::awareness2local>(topic_name,buffersize);
}

void msg_awareness2local::pub_a2l(awareness_map_cylindrical *a_map, ros::Time stamp)
{
    mlmapping::awareness2local a2l_msg;
    a2l_msg.header.stamp = stamp;
    a2l_msg.pt_obs_count = a_map->hit_idx_odds_hashmap.size();
    a2l_msg.pt_miss_count = a_map->miss_idx_set.size();
    cout<<"hit size: "<<a2l_msg.pt_obs_count<<"  miss size: "<<a2l_msg.pt_miss_count<<endl;
    // int i = 0;
    for(auto iter = a_map->hit_idx_odds_hashmap.begin(); iter != a_map->hit_idx_odds_hashmap.end(); iter++)
    {
        geometry_msgs::Vector3 geo_pt;
        Vec3 geo_pt_vec = a_map->map->at(a_map->mapIdx_out(iter->first)).center_pt;
        geo_pt.x = geo_pt_vec[0];
        geo_pt.y = geo_pt_vec[1];
        geo_pt.z = geo_pt_vec[2];
        
        a2l_msg.pts_obs_a.emplace_back(geo_pt);
        a2l_msg.odds_obs_a.emplace_back(log10(iter->second));
    }
    for(size_t idx:a_map->miss_idx_set)
    {
        geometry_msgs::Vector3 geo_pt;
        geo_pt.x = a_map->map->at(idx).center_pt[0];
        geo_pt.y = a_map->map->at(idx).center_pt[1];
        geo_pt.z = a_map->map->at(idx).center_pt[2];
        a2l_msg.pts_miss_a.push_back(geo_pt);
    }
    Vec3 t = a_map->T_wa.translation();
    Quaterniond uq = a_map->T_wa.unit_quaternion();
    a2l_msg.T_w_a.rotation.w=uq.w();
    a2l_msg.T_w_a.rotation.x=uq.x();
    a2l_msg.T_w_a.rotation.y=uq.y();
    a2l_msg.T_w_a.rotation.z=uq.z();
    a2l_msg.T_w_a.translation.x = t(0);
    a2l_msg.T_w_a.translation.y = t(1);
    a2l_msg.T_w_a.translation.z = t(2);
    l2g_pub.publish(a2l_msg);
}

void msg_awareness2local::unpack(mlmapping::awareness2localConstPtr ptr,
                              SE3 &T_w_a,
                              vector<Vec3> &l2g_obs_l,
                              vector<float> &l2g_odds_l,
                              vector<Vec3> &l2g_miss_l,
                              ros::Time &T)
{
    T=ptr->header.stamp;
    Vec3 t;
    Quaterniond uq;
    t(0) = ptr->T_w_a.translation.x;
    t(1) = ptr->T_w_a.translation.y;
    t(2) = ptr->T_w_a.translation.z;
    uq.w() = ptr->T_w_a.rotation.w;
    uq.x() = ptr->T_w_a.rotation.x;
    uq.y() = ptr->T_w_a.rotation.y;
    uq.z() = ptr->T_w_a.rotation.z;
    T_w_a = SE3(uq,t);
    size_t num_obs = ptr->pt_obs_count;
    size_t num_miss = ptr->pt_miss_count;
    for(size_t i=0; i<num_obs; i++)
    {
        l2g_obs_l.push_back(Vec3(ptr->pts_obs_a.at(i).x,ptr->pts_obs_a.at(i).y,ptr->pts_obs_a.at(i).z));
        l2g_odds_l.push_back(ptr->odds_obs_a.at(i));
    }
    for(size_t i=0; i<num_miss; i++)
    {
        l2g_miss_l.push_back(Vec3(ptr->pts_miss_a.at(i).x,ptr->pts_miss_a.at(i).y,ptr->pts_miss_a.at(i).z));
    }
}
