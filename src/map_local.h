#ifndef local_map_cartesian_H
#define local_map_cartesian_H

#include <utils/include/all_utils.h>
#include <data_type.h>
#include <map_warehouse.h>
#include "map_awareness.h"
#define logit(x) (log10((x) / (1 - (x))))
typedef struct SUBMAP_IN_LOCAL_PARA
{
    unsigned int submap_nxy;
    unsigned int submap_nz;
    Vec3 submap_diff_center_offset;
    Vec3 diff_1st_close_submap_center_localmap_min;
    double relevant_submap_search_range_xy;
    double relevant_submap_search_range_z;
} SUBMAP_IN_LOCAL_PARA;

typedef struct SUBMAP_SWITCHING_CHECK_LIST
{
    unsigned int local_sub_map_idx;
    SUBMAP_INFO submap_info;
} SUBMAP_SWITCHING_CHECK_LIST;

typedef struct LOCALMAP_VIS_PARA
{
    double map_size_xy;
    double map_size_z;
    string frame_name;
    double cube_size_xyz;
    double map_minz;
    double map_maxz;
    double map_range_z;
} LOCALMAP_VIS_PARA;

class local_map_cartesian
{
private:
    unsigned int map_nx_times_ny;
    unsigned int unique_submap_idx;

    bool visibility_check;

    float log_odds_hit;
    float log_odds_miss;
    float log_odds_occupied_sh;
    bool first_pose;

    double map_min_x;
    double map_min_y;
    bool local_switch;
    SUBMAP_IN_LOCAL_PARA submap_paras;
    SUBMAP_SWITCHING_CHECK_LIST sub_map_switching_check_list[27]; // cloest 27 submaps
    SUBMAP sub_maps[125];
    int global_min_idxx, global_min_idxy, global_min_idxz, global_max_idxx, global_max_idxy, global_max_idxz;
    bool apply_explored_area;
    vector<Vec3I> nbr_disp;
    vector<Vec3> nbr_disp_real;
    struct VectorHasher
    {
        int operator()(const Vec3I &V) const
        {
            int hash = V.size();
            hash ^= V[0] + 0x9e3779b9 + (hash << 6) + (hash >> 2);
            hash ^= V[1] + 0x9e3779b9 + (hash << 6) + (hash >> 2);
            hash ^= V[2] + 0x9e3779b9 + (hash << 6) + (hash >> 2);
            return hash;
        }
    };
    struct subbox
    {
        vector<char> occupancy;
        vector<float> log_odds;
        unordered_set<int> frontier;
        // vector<Vec3>  odds_grad; //TODO
    };

public:
    float log_odds_max;
    float log_odds_min;
    SE3 T_wa_latest;
    SE3 T_wl; // Transformation from local to world
    double map_min_z;
    Vec3 map_min_xyz;
    unsigned int map_nxy;
    unsigned int map_nz;
    double map_dxyz;
    double map_dxyz_obv_glb, map_dxyz_obv_sub,map_dxyz_obv_sub_half;
    size_t cell_num_subbox;
    int subbox_nxyz;
    int ram_expand_cnt = 0;
    int obs_cnt = 0;
    float map_reso_inv;
    Vec3 map_center_xyz;
    vector<double> global_bd = vector<double>(6);
    vector<unsigned int> occupied_cell_idx;
    // vector<vector<vector<size_t> > > subbox_cell_id_tab;
    unordered_map<Vec3I, size_t, VectorHasher> subbox_cell_id_table;
    vector<Vec3I> subbox_id2xyz_table;
    // unordered_set<int> occupied_cell_idx_map;
    // unordered_map<Vec3I, bool, VectorHasher> frontier_cell_global_idx_map;
    // unordered_map<Vec3I, bool, VectorHasher> observed_cell_global_idx_map;
    // unordered_map<Vec3I, unordered_set<int>, VectorHasher> frontier_group_map;
    unordered_map<Vec3I, subbox, VectorHasher> observed_group_map;
    vector<Mat6x4I> subbox_neighbors; // for each line in matrix, the first 3 elements are the neighbor global id, and the last is the subbox id
    unordered_set<Vec3I, VectorHasher> observed_subboxes;
    // key is the origin of sub-box in the global map. Every cell of the box is stored in the observed container, and only the frontier
    //  cell is stored in the frontier container
    std::unique_ptr<vector<CARTESIAN_CELL>> map;
    LOCALMAP_VIS_PARA vis_paras;

    local_map_cartesian();
    void init_map(double d_xyz_in,
                  unsigned int subbox_n,
                  float log_odds_min_in,
                  float log_odds_max_in,
                  float log_odds_hit_in,
                  float log_odds_miss_in,
                  float log_odds_occupied_sh_in,
                  bool if_apply_explor);
    void clear_map();
    void allocate_memory_for_local_map();
    void devide_local_map_to_submaps();
    bool get_the_relevant_submap_for_new_localmap(vector<unsigned int> &relevant_and_occupied_idx,
                                                  vector<unsigned int> &unrelevant_and_occupied_idx);
    void input_pc_pose_direct(awareness_map_cylindrical *a_map,
                              map_warehouse *warehouse);
    Vec3I xyz2xyzIdx(Vec3 xyz_w);
    bool xyz2xyzIdxwithBoderCheck(Vec3 xyz_w, Vec3I &xyz_idx);
    inline size_t mapIdx(Vec3I xyz_idx);
    inline size_t mapIdx(unsigned int x_idx, unsigned int y_idx, unsigned int z_idx);
    inline Vec3I global_xyzidx(size_t local_idx);
    inline Vec3I global_xyzidx(size_t local_idx, Vec3 &pt_w);
    inline bool inside_exp_bd(Vec3 pt_w);
    void update_observation(Vec3I glb_idx, size_t subbox_id, Vec3 pt_w);
    inline size_t get_subbox_id(Vec3 pt_w);
    inline size_t get_subbox_id(Vec3 &pt_w, Vec3I &glb_idx);
    inline PointP subbox_id2xyz_glb(Vec3I origin, int idx);
    inline Vec3 subbox_id2xyz_glb_vec(Vec3I origin, int idx);
    inline void get_global_idx(Vec3 pt_w, Vec3I &glb_idx, size_t &subbox_id);
    inline bool allocate_ram(Vec3I &glb_idx);
};
inline size_t local_map_cartesian::mapIdx(Vec3I xyz_idx)
{
    return mapIdx(static_cast<unsigned int>(xyz_idx(0)),
                  static_cast<unsigned int>(xyz_idx(1)),
                  static_cast<unsigned int>(xyz_idx(2)));
}
inline size_t local_map_cartesian::mapIdx(unsigned int x_idx, unsigned int y_idx, unsigned int z_idx)
{
    return static_cast<size_t>(z_idx * map_nx_times_ny + y_idx * map_nxy + x_idx);
}

inline Vec3I local_map_cartesian::global_xyzidx(size_t local_idx, Vec3 &pt_w)
{
    pt_w = map->at(local_idx).center_pt + map_center_xyz;
    return Vec3I(floor(pt_w[0] / map_dxyz_obv_glb), floor(pt_w[1] / map_dxyz_obv_glb), floor(pt_w[2] / map_dxyz_obv_glb));
}

inline void local_map_cartesian::get_global_idx(Vec3 pt_w, Vec3I &glb_idx, size_t &subbox_id)
{
    glb_idx = Vec3I(floor(pt_w[0] / map_dxyz_obv_glb), floor(pt_w[1] / map_dxyz_obv_glb), floor(pt_w[2] / map_dxyz_obv_glb));
    subbox_id = get_subbox_id(pt_w, glb_idx);
}

inline Vec3I local_map_cartesian::global_xyzidx(size_t local_idx)
{
    Vec3 pt_w = map->at(local_idx).center_pt + map_center_xyz;
    return Vec3I(static_cast<int>(pt_w[0] / map_dxyz_obv_glb), static_cast<int>(pt_w[1] / map_dxyz_obv_glb), static_cast<int>(pt_w[2] / map_dxyz_obv_glb));
}

inline bool local_map_cartesian::inside_exp_bd(Vec3 pt_w)
{
    return (pt_w[0] >= global_bd[0] && pt_w[0] < global_bd[1] &&
            pt_w[1] >= global_bd[2] && pt_w[1] < global_bd[3] &&
            pt_w[2] >= global_bd[4] && pt_w[2] < global_bd[5]);
}

inline size_t local_map_cartesian::get_subbox_id(Vec3 &pt_w, Vec3I &glb_idx)
{

    return subbox_cell_id_table[Vec3I(floor(pt_w[0] / map_dxyz_obv_sub) - glb_idx[0] * subbox_nxyz,
                                      floor(pt_w[1] / map_dxyz_obv_sub) - glb_idx[1] * subbox_nxyz,
                                      floor(pt_w[2] / map_dxyz_obv_sub) - glb_idx[2] * subbox_nxyz)];
}

inline size_t local_map_cartesian::get_subbox_id(Vec3 pt_w)
{
    return subbox_cell_id_table[Vec3I(static_cast<int>(fmod(pt_w[0], map_dxyz_obv_glb) / map_dxyz_obv_sub),
                                      static_cast<int>(fmod(pt_w[1], map_dxyz_obv_glb) / map_dxyz_obv_sub),
                                      static_cast<int>(fmod(pt_w[2], map_dxyz_obv_glb) / map_dxyz_obv_sub))];
}

inline bool local_map_cartesian::xyz2xyzIdxwithBoderCheck(Vec3 xyz_w, Vec3I &xyz_idx)
{
    double x = xyz_w(0) - map_min_x;
    int x_idx = static_cast<int>(x / map_dxyz);
    double y = xyz_w(1) - map_min_y;
    int y_idx = static_cast<int>(y / map_dxyz);
    double z = xyz_w(2) - map_min_z;
    int z_idx = static_cast<int>(z / map_dxyz);
    if (x > 0 && y > 0 && z > 0)
    {
        if (x > 0 && y > 0 && z > 0 && x_idx >= 0 && y_idx >= 0 && z_idx >= 0 && x_idx < static_cast<int>(map_nxy) && y_idx < static_cast<int>(map_nxy) && z_idx < static_cast<int>(map_nz))
        {
            xyz_idx = Vec3I(x_idx, y_idx, z_idx);
            return true;
        }
    }
    return false;
}

inline PointP local_map_cartesian::subbox_id2xyz_glb(Vec3I origin, int idx)
{
    return PointP(origin[0] * map_dxyz_obv_glb + subbox_id2xyz_table[idx][0] * map_dxyz_obv_sub + map_dxyz_obv_sub_half,
                  origin[1] * map_dxyz_obv_glb + subbox_id2xyz_table[idx][1] * map_dxyz_obv_sub + map_dxyz_obv_sub_half,
                  origin[2] * map_dxyz_obv_glb + subbox_id2xyz_table[idx][2] * map_dxyz_obv_sub + map_dxyz_obv_sub_half);
}

inline Vec3 local_map_cartesian::subbox_id2xyz_glb_vec(Vec3I origin, int idx)
{
    return Vec3(origin[0] * map_dxyz_obv_glb + subbox_id2xyz_table[idx][0] * map_dxyz_obv_sub + map_dxyz_obv_sub_half,
                  origin[1] * map_dxyz_obv_glb + subbox_id2xyz_table[idx][1] * map_dxyz_obv_sub + map_dxyz_obv_sub_half,
                  origin[2] * map_dxyz_obv_glb + subbox_id2xyz_table[idx][2] * map_dxyz_obv_sub + map_dxyz_obv_sub_half);
}

inline bool local_map_cartesian::allocate_ram(Vec3I &glb_idx)
{
    if (observed_group_map.find(glb_idx) == observed_group_map.end()) // if current point belongs to a new group
    {
        observed_group_map[glb_idx].occupancy.resize(cell_num_subbox, 'u'); // u for unknown
        observed_group_map[glb_idx].log_odds.resize(cell_num_subbox, 0);
        observed_group_map[glb_idx].frontier.clear();
        ram_expand_cnt++;
        return true;
    }

    else if (observed_group_map[glb_idx].occupancy.size() == 1)
        return false;

    return true;
}

// inline Vec3 local_map_cartesian::get_pos_world (Vec3I &glb_idx, size_t subbox_id)
// {
//  glb_idx
// }
#endif // local_map_cartesian_H
