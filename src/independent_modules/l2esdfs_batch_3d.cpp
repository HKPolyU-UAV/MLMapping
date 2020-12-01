#include "l2esdfs_batch_3d.h"


Local2ESDFsBatch::Local2ESDFsBatch(ros::NodeHandle& nh, string topic_name, int buffersize)
{
  esdf_pub = nh.advertise<visualization_msgs::Marker>("/esdfs_cubes", buffersize);
  esdf_pc_pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name, buffersize);
}

void Local2ESDFsBatch::setLocalMap(local_map_cartesian *map,
                                   string awareness_frame_name,
                                   int batch_size_nxy_in,
                                   int batch_size_nz_in,
                                   int batch_max_search_range_in,
                                   double esdfs_batch_occupied_in)
{
  map3d_nx = map3d_ny = map->map_nxy;
  map3d_nz = map->map_nz;
  map3d_dx = map3d_dy = map3d_dz = map->map_dxyz;



  batch_size_nxy = batch_size_nxy_in;
  batch_size_nz = batch_size_nz_in;

  this->visualize_min_xyz = Vec3(map3d_dx*(-0.5-((batch_size_nxy-1)/2)),
                                 map3d_dy*(-0.5-((batch_size_nxy-1)/2)),
                                 map3d_dz*(-0.5-((batch_size_nz-1)/2)));

  batch_max_search_range = batch_max_search_range_in;
  esdfs_batch_occupied = esdfs_batch_occupied_in;

  awareness_frame_id = awareness_frame_name;
  visualized_layer = 0;
  cubes_array.header.frame_id  = awareness_frame_name;
  cubes_array.ns = "points";
  cubes_array.type = visualization_msgs::Marker::CUBE_LIST;
  cubes_array.action = visualization_msgs::Marker::ADD;
  cubes_array.pose.orientation.w =  1.0;
  cubes_array.scale.x = cubes_array.scale.y = map3d_dx;
  cubes_array.scale.z = 0.1;//a layer
  cubes_array.id = 0;
  cubes_array.points.clear();
  cubes_array.colors.clear();
  cout << "esdf map" << endl;

  esdf_cutoff_value = batch_max_search_range*map3d_dx;
  for (int i=0; i<batch_size_nz; i++)
  {
    MatrixXd esdf_layer;
    esdf_layer.resize(batch_size_nxy,batch_size_nxy);
    esdf_layer.fill(esdf_cutoff_value);
    esdf_map3d.push_back(esdf_layer);
  }
  cout << "esdf map init finished~" << endl;
}

Vec3 Local2ESDFsBatch::esdf_cube_coler(double ratio)
{
  int normalized = int(ratio * 256 * 5);

  //find the distance to the start of the closest region
  int x = normalized % 256;

  int red = 0, grn = 0, blu = 0;
  switch(normalized / 256)
  {
  case 0: red = 255;      grn = x;        blu = 0;       break;//red
  case 1: red = 255 - x;  grn = 255;      blu = 0;       break;//yellow
  case 2: red = 0;        grn = 255;      blu = x;       break;//green
  case 3: red = 0;        grn = 255 - x;  blu = 255;     break;//cyan
  case 4: red = x;        grn = 0;        blu = 255;     break;//blue
    //case 5: red = 255;      grn = 0;        blu = 255 - x; break;//magenta
  }
  return Vec3(red/256.0,grn/256.0,blu/256.0);
}

Vec3I Local2ESDFsBatch::esdf_cube_coler_int(double ratio)
{
  int normalized = int(ratio * 256 * 5);

  //find the distance to the start of the closest region
  int x = normalized % 256;

  int red = 0, grn = 0, blu = 0;
  switch(normalized / 256)
  {
  case 0: red = 255;      grn = x;        blu = 0;       break;//red
  case 1: red = 255 - x;  grn = 255;      blu = 0;       break;//yellow
  case 2: red = 0;        grn = 255;      blu = x;       break;//green
  case 3: red = 0;        grn = 255 - x;  blu = 255;     break;//cyan
  case 4: red = x;        grn = 0;        blu = 255;     break;//blue
    //case 5: red = 255;      grn = 0;        blu = 255 - x; break;//magenta
  }
  return Vec3I(red,grn,blu);
}

void Local2ESDFsBatch::pub_ESDF_3D_from_localmap(local_map_cartesian *map, ros::Time stamp)
{
  for (size_t i=0; i<batch_size_nz; i++)
  {
    esdf_map3d.at(i).fill(esdf_cutoff_value);
  }

  int idx_min_x, idx_min_y, idx_min_z;
  int idx_max_x, idx_max_y, idx_max_z;
  Vec3 translation_w_a = map->T_wa_latest.translation();
  Vec3I xyz_idx=map->xyz2xyzIdx(translation_w_a);

  idx_min_x = xyz_idx(0)-((batch_size_nxy-1)/2);
  idx_min_y = xyz_idx(1)-((batch_size_nxy-1)/2);
  idx_min_z = xyz_idx(2)-((batch_size_nz-1)/2);
  idx_max_x = xyz_idx(0)+((batch_size_nxy-1)/2);
  idx_max_y = xyz_idx(1)+((batch_size_nxy-1)/2);
  idx_max_z = xyz_idx(2)+((batch_size_nz-1)/2);

  cout << "idx_min_x " << idx_min_x << "idx_max_x " << idx_max_x << endl;
  cout << "idx_min_y " << idx_min_y << "idx_max_y " << idx_max_y << endl;
  cout << "idx_min_z " << idx_min_z << "idx_max_z " << idx_max_z << endl;

  for(auto idx:map->occupied_cell_idx)
  {
    int x = map->map->at(idx).idx_x;
    int y = map->map->at(idx).idx_y;
    int z = map->map->at(idx).idx_z;
    if(     x>=idx_min_x && y>=idx_min_y && z>=idx_min_z &&
            x<=idx_max_x && y<=idx_max_y && z<=idx_max_z)
    {
      //calculate idx in batch
      int idx_x = x-idx_min_x;
      int idx_y = y-idx_min_y;
      int idx_z = z-idx_min_z;
      esdf_map3d.at(static_cast<size_t>(idx_z))(idx_x,idx_y)=esdfs_batch_occupied;
    }
  }
  cout << "extract occupied cells" << endl;
  //calculate the esdf for each layer
  for(int z=0; z<batch_size_nz; z++)
  {
    //    cout << "in layer" << z << endl;
    for (int x = 0; x < batch_size_nxy; x++)
    {
      for (int y = 0; y < batch_size_nxy; y++)
      {
        //cout << "in x " << x << " in y " << y << " in z " << z << endl;
        if(esdf_map3d.at(z)(x,y) < 0.1)
        {
          //          cout << "here" << endl;
          //          cout << "batch_max_search_range" << batch_max_search_range << endl;
          //          cout << "x-batch_max_search_range" << x-batch_max_search_range << endl;
          //          cout << "x+(batch_max_search_range+1)" << x+(batch_max_search_range+1) << endl;
          //          cout << "y-batch_max_search_range" << y-batch_max_search_range << endl;
          //          cout << "y+(batch_max_search_range+1)" << y+(batch_max_search_range+1) << endl;
          for(int xx=x-batch_max_search_range; xx<x+(batch_max_search_range+1); xx++)
          {
            for(int yy=y-batch_max_search_range; yy<y+(batch_max_search_range+1); yy++)
            {
              if(     yy<0 || yy>(batch_size_nxy-1)
                      || xx<0 || xx>(batch_size_nxy-1))
              {//out of range
                //cout << "xx yy out of range" << endl;
                continue;
              }
              if(esdf_map3d.at(z)(xx,yy)==esdfs_batch_occupied)
              {
                continue;
              }
              double dis = sqrt(pow((abs(xx-x)*map3d_dx),2)+pow((abs(yy-y)*map3d_dy),2));
              if(dis>esdf_cutoff_value) dis=esdf_cutoff_value;
              if(esdf_map3d.at(z)(xx,yy)>dis)
              {
                esdf_map3d.at(z)(xx,yy)=dis;
              }
            }
          }
        }
      }
    }
  }
  cout << "calculated single layer" << endl;
  //   integrate update to multiple layer
  MatrixXd increase_dz;
  increase_dz.resize(batch_size_nxy,batch_size_nxy);
  increase_dz.fill(map3d_dz);
  for(int loop=0; loop<batch_max_search_range-1; loop++)
  {
    // go up
    for(size_t layer=0; layer<batch_size_nz-1; layer++)
    {
      MatrixXd update_matrix = esdf_map3d.at(layer)+increase_dz;
      MatrixXd residual_matrix = update_matrix-esdf_map3d.at(layer+1);
      MatrixXb update_mask_matrix = residual_matrix.unaryExpr([](double d){ return d < 0.0; });
      for (int x = 0; x < batch_size_nxy; x++)
      {
        for (int y = 0; y < batch_size_nxy; y++)
        {
          if(update_mask_matrix(x,y)==true)
          {
            esdf_map3d.at(layer+1)(x,y) = update_matrix(x,y);
          }
        }
      }
    }
    // go down
    for(size_t layer=batch_size_nz-1; layer>1; layer--)
    {
      MatrixXd update_matrix = esdf_map3d.at(layer)+increase_dz;
      MatrixXd residual_matrix = update_matrix-esdf_map3d.at(layer-1);
      MatrixXb update_mask_matrix = residual_matrix.unaryExpr([](double d){ return d < 0.0; });
      for (int x = 0; x < batch_size_nxy; x++)
      {
        for (int y = 0; y < batch_size_nxy; y++)
        {
          if(update_mask_matrix(x,y)==true)
          {
            esdf_map3d.at(layer-1)(x,y) = update_matrix(x,y);
          }
        }
      }
    }
  }

  double max_esdf_range = esdf_cutoff_value*1.05;



  sensor_msgs::PointCloud2 output;
  PointCloudRGB_ptr pc (new PointCloudRGB);
  cout << "awareness_frame_id " << awareness_frame_id << endl;
  pc->header.frame_id = this->awareness_frame_id;
  pc->height = 1;
  for( int layer = 0; layer < batch_size_nz; layer++)
  {
    double visulized_height = visualize_min_xyz(2) +(layer)*map3d_dz;
    for (int x = 0; x < batch_size_nxy; x++)
    {
      for (int y = 0; y < batch_size_nxy; y++)
      {
        if(esdf_map3d.at(layer)(x,y) == esdfs_batch_occupied)
        {//Occupied ->BLACK
          PointRGB p;
          p.x = visualize_min_xyz(0)+x*this->map3d_dx;
          p.y = visualize_min_xyz(1)+y*this->map3d_dy;
          p.z = visulized_height;
          uint8_t r = 0, g = 0, b = 0;    // Example: Red color
          uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
          p.rgb = *reinterpret_cast<float*>(&rgb);
          pc->points.push_back(p);
        }else
        {//Color according to distance
          //        pc->points.push_back (PointP(visualize_min_xyz(0)+x*this->map3d_dx,
          //                                     visualize_min_xyz(1)+y*this->map3d_dy,
          //                                     visualize_min_xyz(2)+1*this->map3d_dz));
          double ratio = esdf_map3d.at(layer)(x,y)/max_esdf_range;
          Vec3I esdf_rgb = this->esdf_cube_coler_int(ratio);
          std_msgs::ColorRGBA color;
          PointRGB p;
          p.x = visualize_min_xyz(0)+x*this->map3d_dx;
          p.y = visualize_min_xyz(1)+y*this->map3d_dy;
          p.z = visulized_height;
          uint8_t r = esdf_rgb(0), g = esdf_rgb(1), b = esdf_rgb(2);    // Example: Red color
          uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
          p.rgb = *reinterpret_cast<float*>(&rgb);
          pc->points.push_back(p);
        }
      }
    }
  }
  pc->width = pc->points.size();
  pcl::toROSMsg(*pc , output);
  output.header.stamp = stamp;
  esdf_pc_pub.publish (output);

//  cout << "multi layer" << endl;
//  cubes_array.points.clear();
//  cubes_array.colors.clear();
//  cubes_array.header.stamp = stamp;
//  double visulized_height;
//  visulized_height = visualize_min_xyz(2) +(visualized_layer)*map3d_dz;
//  MatrixXd esdf_vis_layer = esdf_map3d.at(visualized_layer);
//  visualized_layer++;
//  if(visualized_layer == batch_size_nz) visualized_layer=0;
//  for (int x = 0; x < batch_size_nxy; x++)
//  {
//    for (int y = 0; y < batch_size_nxy; y++)
//    {
//      if(esdf_vis_layer(x,y) == ESDF_BATCH_UNKNOWN)
//      {//Unknown ->Nothing
//        geometry_msgs::Point point;
//        point.x = mat2vismap_bias_x+x*this->map3d_dx;
//        point.y = mat2vismap_bias_y+y*this->map3d_dy;
//        point.z = visulized_height;
//        this->cubes_array.points.push_back(point);
//        std_msgs::ColorRGBA color;
//        color.r= 0.0;
//        color.g= 0.8;
//        color.b= 0.0;
//        color.a= static_cast<float>(0.1);
//        this->cubes_array.colors.push_back(color);
//        continue;
//      }
//      if(esdf_vis_layer(x,y) == esdfs_batch_occupied)
//      {//Occupied ->BLACK
//        geometry_msgs::Point point;
//        point.x = visualize_min_xyz(0)+x*this->map3d_dx;
//        point.y = visualize_min_xyz(1)+y*this->map3d_dy;
//        point.z = visulized_height;
//        this->cubes_array.points.push_back(point);
//        std_msgs::ColorRGBA color;
//        color.r= 0.0;
//        color.g= 0.0;
//        color.b= 0.0;
//        color.a= static_cast<float>(0.9);
//        this->cubes_array.colors.push_back(color);
//      }else
//      {//Color according to distance
//        geometry_msgs::Point point;
//        point.x = visualize_min_xyz(0)+x*this->map3d_dx;
//        point.y = visualize_min_xyz(1)+y*this->map3d_dy;
//        point.z = visulized_height;
//        this->cubes_array.points.push_back(point);
//        double ratio = esdf_vis_layer(x,y)/max_esdf_range;
//        Vec3 rgb = this->esdf_cube_coler(ratio);
//        std_msgs::ColorRGBA color;
//        color.r= static_cast<float>(rgb(0));
//        color.g= static_cast<float>(rgb(1));
//        color.b= static_cast<float>(rgb(2));
//        color.a= static_cast<float>(0.9);
//        this->cubes_array.colors.push_back(color);
//      }
//    }
//  }
//  cout << "visualization" << endl;
//  if(cubes_array.points.size()!=0)
//  {
//    this->esdf_pub.publish(cubes_array);
//  }
}
