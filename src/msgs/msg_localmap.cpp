#include "msg_localmap.h"

msg_localmap::msg_localmap()
{

}

msg_localmap::msg_localmap(ros::NodeHandle& nh, string topic_name, int buffersize)
{
    this->localmap_pub = nh.advertise<mlmapping::localmap>(topic_name,buffersize);
}

void msg_localmap::pub(local_map_cartesian *map,
                       ros::Time stamp)
{
    mlmapping::localmap msg;
    msg.header.stamp = stamp;
    msg.occupied_cell_count = static_cast<unsigned int>(map->occupied_cell_idx.size());
    for(auto cell: map->occupied_cell_idx)
    {
        msg.occupied_cell_idx.push_back(cell);
    }
    Vec3 t = map->T_wl.translation();
    Quaterniond uq= map->T_wl.unit_quaternion();
    msg.T_w_l.rotation.w=uq.w();
    msg.T_w_l.rotation.x=uq.x();
    msg.T_w_l.rotation.y=uq.y();
    msg.T_w_l.rotation.z=uq.z();
    msg.T_w_l.translation.x = t(0);
    msg.T_w_l.translation.y = t(1);
    msg.T_w_l.translation.z = t(2);
    localmap_pub.publish(msg);
}

void msg_localmap::unpack(mlmapping::localmapConstPtr msg_ptr,
                          local_map_cartesian *map_output)
{
    map_output->T_wl = SE3(Quaterniond(msg_ptr->T_w_l.rotation.w,
                                msg_ptr->T_w_l.rotation.x,
                                msg_ptr->T_w_l.rotation.y,
                                msg_ptr->T_w_l.rotation.z),
                    Vector3d(msg_ptr->T_w_l.translation.x,
                             msg_ptr->T_w_l.translation.y,
                             msg_ptr->T_w_l.translation.z));
    map_output->clear_map();
    size_t cnt = msg_ptr->occupied_cell_count;
    for(size_t i=0; i<cnt; i++)
    {
        map_output->occupied_cell_idx.push_back(msg_ptr->occupied_cell_idx.at(i));
        map_output->map->at(i).is_occupied=true;
    }
}
