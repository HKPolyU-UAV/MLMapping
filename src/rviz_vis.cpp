#include "rviz_vis.h"


rviz_vis::rviz_vis()
{

}

rviz_vis::~rviz_vis()
{

}

void rviz_vis::set_as_awareness_map_publisher(ros::NodeHandle& nh,
                         string topic_name,
                         string frame_id,
                         unsigned int buffer_size,
                         awareness_map_cylindrical* awareness_map)
{
    this->map_pub = nh.advertise<visualization_msgs::Marker>(topic_name, buffer_size);
    this->frame_id = frame_id;
    this->min_z = awareness_map->z_border_min;
    this->max_z = awareness_map->z_border_min + (awareness_map->map_dZ*awareness_map->map_nZ);
    this->range_z = max_z-min_z;
}

void rviz_vis::set_as_local_map_publisher(ros::NodeHandle& nh,
                   string topic_name,
                   string frame_id,
                   unsigned int buffer_size,
                   local_map_cartesian* localmap)
{
    this->map_pub = nh.advertise<visualization_msgs::Marker>(topic_name, buffer_size);
    this->frame_id = frame_id;
    this->cube_size_xyz = localmap->vis_paras.cube_size_xyz;
    this->min_z = localmap->vis_paras.map_minz;
    this->max_z = localmap->vis_paras.map_maxz;
    this->range_z = max_z-min_z;
}

void rviz_vis::set_as_global_map_publisher(ros::NodeHandle& nh,
                   string topic_name,
                   string frame_id,
                   unsigned int buffer_size)
{
    this->map_pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name, buffer_size);
    this->frame_id = frame_id;
}

//input: ratio is between 0 to 1
//output: rgb color
Vec3 sphereColer(double ratio)
{
    //we want to normalize ratio so that it fits in to 6 regions
    //where each region is 256 units long
    int normalized = int(ratio * 256 * 3);
    //find the distance to the start of the closest region
    int x = normalized % 256;
    int red = 0, grn = 0, blu = 0;
    switch(normalized / 256)
    {
    case 0: red = 255;      grn = x;        blu = 0;       break;//red->yellow
    case 1: red = 255 - x;  grn = 255;      blu = 0;       break;//yellow->green
    case 2: red = 0;        grn = 255;      blu = x;       break;//green->cyan
    //case 3: red = 0;        grn = 255 - x;  blu = 255;     break;//cyan->blue
    //case 4: red = x;        grn = 0;        blu = 255;     break;//blue
    }

    return Vec3(red/260.0,grn/260.0,blu/260.0);
}

Vec3 cubeColer(double ratio)
{
    //we want to normalize ratio so that it fits in to 6 regions
    //where each region is 256 units long
    int normalized = int(ratio * 256 * 3);
    //find the distance to the start of the closest region
    int x = normalized % 256;
    int red = 0, grn = 0, blu = 0;
    switch(normalized / 256)
    {
    case 0: red = 0;        grn = 255 - x;  blu = 255;     break;//cyan->blue
    case 1: red = x;        grn = 0;        blu = 255;     break;//blue->magenta
    case 2: red = 255;      grn = 0;        blu = 255 - x; break;//magenta->red
    }

    return Vec3(red/260.0,grn/260.0,blu/260.0);
}

void rviz_vis::pub_awareness_map(awareness_map_cylindrical* localmap, const ros::Time stamp)
{
    visualization_msgs::Marker spheres;
    spheres.header.frame_id  = this->frame_id;
    spheres.header.stamp = stamp;
    spheres.ns = "points";
    spheres.type = visualization_msgs::Marker::SPHERE_LIST;
    spheres.action = visualization_msgs::Marker::ADD;
    spheres.pose.orientation.w =  1.0;
    spheres.scale.x = spheres.scale.y = spheres.scale.z = 0.4;
    spheres.id = 0;
    for (auto i:localmap->occupied_cell_idx) {
        geometry_msgs::Point point;
        Vec3 pt = localmap->map->at(i).center_pt;
        point.x = pt.x();
        point.y = pt.y();
        point.z = pt.z();
        spheres.points.push_back(point);
        std_msgs::ColorRGBA color;
//        double ratio = (pt.z()-min_z)/range_z;
//        Vec3 rgb = sphereColer(ratio);
//        color.r= static_cast<float>(rgb(0));
//        color.g= static_cast<float>(rgb(1));
//        color.b= static_cast<float>(rgb(2));
//        color.a= static_cast<float>(0.9);
        color.r= static_cast<float>(1.0);
        color.g= static_cast<float>(1.0);
        color.b= static_cast<float>(1.0);
        color.a= static_cast<float>(1.0);
        spheres.colors.push_back(color);
    }
    if(spheres.points.size()!=0)
    {
        this->map_pub.publish(spheres);
    }
}

void rviz_vis::pub_local_map(local_map_cartesian* localmap, const ros::Time stamp)
{
    visualization_msgs::Marker cubes;
    cubes.header.frame_id  = this->frame_id;
    cubes.header.stamp = stamp;
    cubes.ns = "points";
    cubes.type = visualization_msgs::Marker::CUBE_LIST;
    cubes.action = visualization_msgs::Marker::ADD;
    cubes.pose.orientation.w =  1.0;
    cubes.scale.x = cubes.scale.y = cubes.scale.z = cube_size_xyz;
    cubes.id = 0;
    //cout << "localmap occupied_cell_idx size " << localmap->occupied_cell_idx.size() << endl;
    for (auto i:localmap->occupied_cell_idx) {
        geometry_msgs::Point point;
        Vec3 pt = localmap->map->at(i).center_pt;
        point.x = pt(0);
        point.y = pt(1);
        point.z = pt(2);
        cubes.points.push_back(point);
        double ratio = (pt.z()-min_z)/range_z;
        Vec3 rgb = cubeColer(ratio);
        std_msgs::ColorRGBA color;
        color.r= static_cast<float>(rgb(0));
        color.g= static_cast<float>(rgb(1));
        color.b= static_cast<float>(rgb(2));
        color.a= static_cast<float>(0.5);
        cubes.colors.push_back(color);
    }
    if(cubes.points.size()!=0)
    {
        this->map_pub.publish(cubes);
    }
    //publish the range
    visualization_msgs::Marker range;
    range.header.frame_id = this->frame_id;
    range.header.stamp = ros::Time();
    range.ns = "range";
    range.id = 0;
    range.type = visualization_msgs::Marker::CUBE;
    range.action = visualization_msgs::Marker::ADD;
    range.pose.position.x = range.pose.position.y = range.pose.position.z = 0;
    range.pose.orientation.x = 0.0;
    range.pose.orientation.y = 0.0;
    range.pose.orientation.z = 0.0;
    range.pose.orientation.w = 1.0;
    range.scale.x = range.scale.y = localmap->vis_paras.map_size_xy;
    range.scale.z = localmap->vis_paras.map_size_z;
    range.color.a = 0.1; // Don't forget to set the alpha!
    range.color.r = 0.0;
    range.color.g = 1.0;
    range.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    this->map_pub.publish(range);
    visualization_msgs::Marker range2=range;
    visualization_msgs::Marker range3=range;
    range2.id = 1;
    range2.scale.x = range2.scale.y = localmap->vis_paras.map_size_xy*0.2;
    range2.scale.z = localmap->vis_paras.map_size_z*0.2;
    range2.color.a = 0.2;
    this->map_pub.publish(range2);
    range3.id = 2;
//    range3.scale.x = range3.scale.y = localmap->vis_paras.map_size_xy*0.6;
//    range3.scale.z = localmap->vis_paras.map_size_z*0.6;
//    range3.color.a = 0.2;
//    this->map_pub.publish(range3);

}

//typedef pcl::PointXYZ             PointP;
//typedef pcl::PointCloud<PointP>   PointCloudP;
//typedef PointCloudP::Ptr          PointCloudP_ptr;

void rviz_vis::pub_global_map(map_warehouse* warehouse,
                              const ros::Time stamp)
{
    //cout << "publish global map" << endl;
    sensor_msgs::PointCloud2 output;
    PointCloudP_ptr pc (new PointCloudP);
    pc->header.frame_id = this->frame_id;
    pc->height = 1;
    for(auto submap:warehouse->warehouse)
    {
        for(auto cell:submap.cells)
        {
            pc->points.push_back (PointP(cell.pt_w.x(), cell.pt_w.y(), cell.pt_w.z()));
        }
    }
    pc->width = pc->points.size();
    pcl::toROSMsg(*pc , output);
    output.header.stamp = stamp;
    map_pub.publish (output);
}


void rviz_vis::pub_global_local_map(map_warehouse* warehouse,
                                    local_map_cartesian* localmap,
                                    const ros::Time stamp)
{
    //cout << "publish global map" << endl;
    sensor_msgs::PointCloud2 output;
    PointCloudP_ptr pc (new PointCloudP);
    pc->header.frame_id = this->frame_id;
    pc->height = 1;
    for(auto submap:warehouse->warehouse)
    {
        for(auto cell:submap.cells)
        {
            pc->points.push_back (PointP(cell.pt_w.x(), cell.pt_w.y(), cell.pt_w.z()));
        }
    }
    Vec3 center_offset = localmap->map_center_xyz;
    for (auto i:localmap->occupied_cell_idx) {
        geometry_msgs::Point point;
        Vec3 pt = localmap->map->at(i).center_pt;
        pt += center_offset;
        pc->points.push_back (PointP(pt.x(),pt.y(),pt.z()));
    }

    pc->width = pc->points.size();
    pcl::toROSMsg(*pc , output);
    output.header.stamp = stamp;
    map_pub.publish (output);
}
