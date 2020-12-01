#include "msg_awareness2local.h"

msg_awareness2local::msg_awareness2local()
{

}

msg_awareness2local::msg_awareness2local(ros::NodeHandle& nh, string topic_name, int buffersize)
{
    l2g_pub = nh.advertise<mlmapping::awareness2local>(topic_name,buffersize);
}

void msg_awareness2local::pub(const SE3 T_w_a, vector<Vec3> obs_pts, vector<Vec3> miss_pts, ros::Time stamp)
{
    mlmapping::awareness2local a2l_msg;
    a2l_msg.header.stamp = stamp;
    a2l_msg.pt_obs_count = obs_pts.size();
    a2l_msg.pt_miss_count = miss_pts.size();
    for(Vec3 pt:obs_pts)
    {
        geometry_msgs::Vector3 geo_pt;
        geo_pt.x = pt(0);
        geo_pt.y = pt(1);
        geo_pt.z = pt(2);
        a2l_msg.pts_obs_a.push_back(geo_pt);
    }
    for(Vec3 pt:miss_pts)
    {
        geometry_msgs::Vector3 geo_pt;
        geo_pt.x = pt(0);
        geo_pt.y = pt(1);
        geo_pt.z = pt(2);
        a2l_msg.pts_miss_a.push_back(geo_pt);
    }
    Vec3 t=T_w_a.translation();
    Quaterniond uq= T_w_a.unit_quaternion();
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
    }
    for(size_t i=0; i<num_miss; i++)
    {
        l2g_miss_l.push_back(Vec3(ptr->pts_miss_a.at(i).x,ptr->pts_miss_a.at(i).y,ptr->pts_miss_a.at(i).z));
    }
}
