#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mlmapping/awareness.h>
#include <mlmapping/localmap.h>
#include <msg_awareness.h>
#include <msg_localmap.h>
#include <rviz_vis.h>

awareness_map_cylindrical* awareness_map;
local_map_cartesian*       local_map;
rviz_vis*              awareness_map_rviz_pub;
rviz_vis*              local_map_rviz_pub;

void awarenessmap_msg_callback(const mlmapping::awarenessConstPtr awareness_map_msg)
{
  msg_awareness::unpack(awareness_map_msg,awareness_map);
  awareness_map_rviz_pub->pub_awareness_map(awareness_map,awareness_map_msg->header.stamp);
}

void localmap_msg_callback(const mlmapping::localmapConstPtr local_map_msg)
{
  msg_localmap::unpack(local_map_msg,local_map);
  local_map_rviz_pub->pub_local_map(local_map,local_map_msg->header.stamp);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visulization node");
  ros::NodeHandle nh;
  string configFilePath;
  nh.getParam("/mlmapping_configfile",   configFilePath);
  //init map and publisher
  //awareness_map
  awareness_map = new awareness_map_cylindrical();
  awareness_map->init_map(getDoubleVariableFromYaml(configFilePath,"mlmapping_am_d_Rho"),
                          getDoubleVariableFromYaml(configFilePath,"mlmapping_am_d_Phi_deg"),
                          getDoubleVariableFromYaml(configFilePath,"mlmapping_am_d_Z"),
                          getIntVariableFromYaml(configFilePath,"mlmapping_am_n_Rho"),
                          getIntVariableFromYaml(configFilePath,"mlmapping_am_n_Z_below"),
                          getIntVariableFromYaml(configFilePath,"mlmapping_am_n_Z_over"),false);
  awareness_map->map_tmp.release();
  awareness_map_rviz_pub =  new rviz_vis();
  awareness_map_rviz_pub->set_as_awareness_map_publisher(nh,"/awareness_map",getStringFromYaml(configFilePath,"awareness_frame_id"),3,awareness_map);
  ros::Subscriber sub1 = nh.subscribe("/mlmapping_awareness", 1, awarenessmap_msg_callback);
  //local_map
  local_map = new local_map_cartesian();
  local_map->init_map(getDoubleVariableFromYaml(configFilePath,"mlmapping_lm_d_xyz"),
                      static_cast<unsigned int>(getIntVariableFromYaml(configFilePath,"mlmapping_lm_n_xy")),
                      static_cast<unsigned int>(getIntVariableFromYaml(configFilePath,"mlmapping_lm_n_z")),
                      static_cast<float>(getDoubleVariableFromYaml(configFilePath,"mlmapping_lm_log_odds_min")),
                      static_cast<float>(getDoubleVariableFromYaml(configFilePath,"mlmapping_lm_log_odds_max")),
                      static_cast<float>(getDoubleVariableFromYaml(configFilePath,"mlmapping_lm_measurement_hit")),
                      static_cast<float>(getDoubleVariableFromYaml(configFilePath,"mlmapping_lm_measurement_miss")),
                      static_cast<float>(getDoubleVariableFromYaml(configFilePath,"mlmapping_lm_occupied_sh")));
  local_map->allocate_memory_for_local_map();
  local_map_rviz_pub = new rviz_vis();
  local_map_rviz_pub->set_as_local_map_publisher(nh,"/local_map",
                                                  getStringFromYaml(configFilePath,"local_frame_id"),
                                                  5,
                                                  local_map);
  ros::Subscriber sub2 = nh.subscribe("/mlmapping_local", 1, localmap_msg_callback);
  ros::spin();

  return 0;
}


