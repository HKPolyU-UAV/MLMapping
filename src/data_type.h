#ifndef DATA_TYPE_H
#define DATA_TYPE_H

#include <utils/include/all_utils.h>

struct CYLINDRICAL_CELL{
  unsigned int idx;
  unsigned int idx_rho;
  unsigned int idx_phi;
  unsigned int idx_z;
  Vec3 center_pt;
  Vec3 sampled_xyz;
  double raycasting_z_over_rho;
  bool is_occupied;
};

typedef struct CARTESIAN_CELL{
    unsigned int idx;
    unsigned int idx_x;
    unsigned int idx_y;
    unsigned int idx_z;
    unsigned int project2d_idx;
    unsigned int relevant_submap_idx;
    Vec3  center_pt;//in local frame center
    bool  is_occupied;
    float log_odds;
    float sd;
}CARTESIAN_CELL;

typedef struct SUBMAP_CELL{
    Vec3  pt_w;
    float log_odds;
}SUBMAP_CELL;

typedef struct SUBMAP_INFO{
    unsigned idx;
    Vec3  offset_min_xyz;
    Vec3  center_xyz;
    unsigned int num_occupied_cell;
}SUBMAP_INFO;

typedef struct SUBMAP{
    SUBMAP_INFO submap_info;
    vector<SUBMAP_CELL> cells;
}SUBMAP;

#endif // DATA_TYPE_H
