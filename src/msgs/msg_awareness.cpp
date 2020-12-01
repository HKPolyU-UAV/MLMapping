#include "msg_awareness.h"

msg_awareness::msg_awareness()
{

}

msg_awareness::msg_awareness(ros::NodeHandle& nh, string topic_name, int buffersize)
{
    this->awarenessmap_pub = nh.advertise<mlmapping::awareness>(topic_name,buffersize);
}

void msg_awareness::pub(awareness_map_cylindrical *map, ros::Time stamp)
{
    mlmapping::awareness msg;
    msg.header.stamp = stamp;
    msg.occupied_cell_count = static_cast<unsigned int>(map->occupied_cell_idx.size());
    for(auto cell: map->occupied_cell_idx)
    {
        msg.occupied_cell_idx.push_back(cell);
    }
    Vec3 t = map->T_wa.translation();
    Quaterniond uq= map->T_wa.unit_quaternion();
    msg.T_w_a.rotation.w=uq.w();
    msg.T_w_a.rotation.x=uq.x();
    msg.T_w_a.rotation.y=uq.y();
    msg.T_w_a.rotation.z=uq.z();
    msg.T_w_a.translation.x = t(0);
    msg.T_w_a.translation.y = t(1);
    msg.T_w_a.translation.z = t(2);
    awarenessmap_pub.publish(msg);

}

void msg_awareness::unpack(mlmapping::awarenessConstPtr msg_ptr,
                          awareness_map_cylindrical *map_output)
{
    map_output->T_wa = SE3(Quaterniond(msg_ptr->T_w_a.rotation.w,
                                msg_ptr->T_w_a.rotation.x,
                                msg_ptr->T_w_a.rotation.y,
                                msg_ptr->T_w_a.rotation.z),
                    Vector3d(msg_ptr->T_w_a.translation.x,
                             msg_ptr->T_w_a.translation.y,
                             msg_ptr->T_w_a.translation.z));
    map_output->clear_map();
    size_t cnt = msg_ptr->occupied_cell_count;
    for(size_t i=0; i<cnt; i++)
    {
        map_output->occupied_cell_idx.push_back(msg_ptr->occupied_cell_idx.at(i));
        map_output->map->at(i).is_occupied=true;
    }
}
