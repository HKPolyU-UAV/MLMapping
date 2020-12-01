#ifndef local_map_cartesian_H
#define local_map_cartesian_H

#include <utils/include/all_utils.h>
#include <data_type.h>
#include <map_warehouse.h>


typedef struct SUBMAP_IN_LOCAL_PARA{
    unsigned int submap_nxy;
    unsigned int submap_nz;
    Vec3 submap_diff_center_offset;
    Vec3 diff_1st_close_submap_center_localmap_min;
    double relevant_submap_search_range_xy;
    double relevant_submap_search_range_z;
}SUBMAP_IN_LOCAL_PARA;

typedef struct SUBMAP_SWITCHING_CHECK_LIST{
    unsigned int local_sub_map_idx;
    SUBMAP_INFO submap_info;
}SUBMAP_SWITCHING_CHECK_LIST;

typedef struct LOCALMAP_VIS_PARA{
    double map_size_xy;
    double map_size_z;
    string frame_name;
    double cube_size_xyz;
    double map_minz;
    double map_maxz;
    double map_range_z;
}LOCALMAP_VIS_PARA;

class local_map_cartesian
{
private:
    unsigned int map_nx_times_ny;
    unsigned int unique_submap_idx;

    bool visibility_check;
    float log_odds_max;
    float log_odds_min;
    float log_odds_hit;
    float log_odds_miss;
    float log_odds_occupied_sh;
    bool  first_pose;

    double map_min_x;
    double map_min_y;

    SUBMAP_IN_LOCAL_PARA        submap_paras;
    SUBMAP_SWITCHING_CHECK_LIST sub_map_switching_check_list[27];//cloest 27 submaps
    SUBMAP                      sub_maps[125];

public:
    SE3 T_wa_latest;
    SE3 T_wl; //Transformation from local to world
    double map_min_z;
    Vec3   map_min_xyz;
    unsigned int map_nxy;
    unsigned int map_nz;
    double map_dxyz;
    Vec3   map_center_xyz;
    vector<unsigned int> occupied_cell_idx;
    std::unique_ptr<vector<CARTESIAN_CELL>> map;
    LOCALMAP_VIS_PARA vis_paras;

    local_map_cartesian();
    void init_map(double d_xyz_in,
                  unsigned int n_xy_in,
                  unsigned n_z_in,
                  float log_odds_min_in,
                  float log_odds_max_in,
                  float log_odds_hit_in,
                  float log_odds_miss_in,
                  float log_odds_occupied_sh_in);
    void clear_map();
    void allocate_memory_for_local_map();
    void devide_local_map_to_submaps();
    bool get_the_relevant_submap_for_new_localmap(vector<unsigned int>& relevant_and_occupied_idx,
                                                  vector<unsigned int>& unrelevant_and_occupied_idx);
    void input_pc_pose(vector<Vec3> PC_hit_a,
                       vector<Vec3> PC_miss_a,
                       SE3 T_wa_in,
                       map_warehouse* warehouse);
    Vec3I xyz2xyzIdx(Vec3 xyz_w);
    bool  xyz2xyzIdxwithBoderCheck(Vec3 xyz_w, Vec3I &xyz_idx);
    size_t mapIdx(Vec3I xyz_idx);
    size_t mapIdx(unsigned int x_idx, unsigned int y_idx, unsigned int z_idx);
};

#endif // local_map_cartesian_H
