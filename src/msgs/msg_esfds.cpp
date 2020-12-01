#include "msg_awareness2local.h"

msg_awareness2local::msg_awareness2local()
{

}

msg_awareness2local::msg_awareness2local(ros::NodeHandle& nh, string topic_name, int buffersize)
{
    l2g_pub = nh.advertise<mlmapping::local2global>(topic_name,buffersize);
}

void msg_awareness2local::pub(const SE3 T_w_l, vector<Vec3> obs_pts, vector<Vec3> miss_pts, ros::Time stamp)
{
    mlmapping::local2global l2g_msg;
    l2g_msg.header.stamp = stamp;
    l2g_msg.pt_obs_count = obs_pts.size();
    l2g_msg.pt_miss_count = miss_pts.size();
    for(Vec3 pt:obs_pts)
    {
        geometry_msgs::Vector3 geo_pt;
        geo_pt.x = pt(0);
        geo_pt.y = pt(1);
        geo_pt.z = pt(2);
        l2g_msg.pts_obs_l.push_back(geo_pt);
    }
    for(Vec3 pt:miss_pts)
    {
        geometry_msgs::Vector3 geo_pt;
        geo_pt.x = pt(0);
        geo_pt.y = pt(1);
        geo_pt.z = pt(2);
        l2g_msg.pts_miss_l.push_back(geo_pt);
    }
    Vec3 t=T_w_l.translation();
    Quaterniond uq= T_w_l.unit_quaternion();
    l2g_msg.T_w_l.rotation.w=uq.w();
    l2g_msg.T_w_l.rotation.x=uq.x();
    l2g_msg.T_w_l.rotation.y=uq.y();
    l2g_msg.T_w_l.rotation.z=uq.z();
    l2g_msg.T_w_l.translation.x = t(0);
    l2g_msg.T_w_l.translation.y = t(1);
    l2g_msg.T_w_l.translation.z = t(2);
    l2g_pub.publish(l2g_msg);
}

void msg_awareness2local::unpack(mlmapping::local2globalConstPtr ptr,
                              SE3 &T_w_l,
                              vector<Vec3> &l2g_obs_l,
                              vector<Vec3> &l2g_miss_l,
                              ros::Time &T)
{
    T=ptr->header.stamp;
    Vec3 t;
    Quaterniond uq;
    t(0) = ptr->T_w_l.translation.x;
    t(1) = ptr->T_w_l.translation.y;
    t(2) = ptr->T_w_l.translation.z;
    uq.w() = ptr->T_w_l.rotation.w;
    uq.x() = ptr->T_w_l.rotation.x;
    uq.y() = ptr->T_w_l.rotation.y;
    uq.z() = ptr->T_w_l.rotation.z;
    T_w_l = SE3(uq,t);
    int num_obs = ptr->pt_obs_count;
    int num_miss = ptr->pt_miss_count;
    for(size_t i=0; i<num_obs; i++)
    {
        l2g_obs_l.push_back(Vec3(ptr->pts_obs_l.at(i).x,ptr->pts_obs_l.at(i).y,ptr->pts_obs_l.at(i).z));
    }
    for(size_t i=0; i<num_miss; i++)
    {
        l2g_miss_l.push_back(Vec3(ptr->pts_miss_l.at(i).x,ptr->pts_miss_l.at(i).y,ptr->pts_miss_l.at(i).z));
    }
}
