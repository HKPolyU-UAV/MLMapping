#ifndef local_map_cartesian_H
#define local_map_cartesian_H

#include <utils/include/all_utils.h>
#include <data_type.h>
#include <map_warehouse.h>

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
    float log_odds_max;
    float log_odds_min;
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

public:
    SE3 T_wa_latest;
    SE3 T_wl; // Transformation from local to world
    double map_min_z;
    Vec3 map_min_xyz;
    unsigned int map_nxy;
    unsigned int map_nz;
    double map_dxyz;
    double map_dxyz_obv_glb, map_dxyz_obv_sub;
    size_t cell_num_subbox;
    int subbox_nxyz;
    int ram_expand_cnt = 0;
    Vec3 map_center_xyz;
    vector<double> global_bd = vector<double>(6);
    vector<unsigned int> occupied_cell_idx;
    // vector<vector<vector<size_t> > > subbox_cell_id_tab;
    unordered_map<Vec3I, size_t, VectorHasher> subbox_cell_id_table;
    vector<Vec3I> subbox_id2xyz_table;
    unordered_map<int, bool> occupied_cell_idx_map;
    unordered_map<Vec3I, bool, VectorHasher> frontier_cell_global_idx_map;
    unordered_map<Vec3I, bool, VectorHasher> observed_cell_global_idx_map;
    unordered_map<Vec3I, unordered_set<int>, VectorHasher> frontier_group_map;
    unordered_map<Vec3I, vector<bool>, VectorHasher> observed_group_map;
    unordered_set<Vec3I, VectorHasher> observed_subboxes;
    // key is the origin of sub-box in the global map. Every cell of the box is stored in the observed container, and only the frontier
    //  cell is stored in the frontier container
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
                  float log_odds_occupied_sh_in,
                  bool if_apply_explor);
    void clear_map();
    void allocate_memory_for_local_map();
    void devide_local_map_to_submaps();
    bool get_the_relevant_submap_for_new_localmap(vector<unsigned int> &relevant_and_occupied_idx,
                                                  vector<unsigned int> &unrelevant_and_occupied_idx);
    void input_pc_pose(vector<Vec3> PC_hit_a,
                       vector<float> PC_odds_a,
                       vector<Vec3> PC_miss_a,
                       SE3 T_wa_in,
                       map_warehouse *warehouse);
    Vec3I xyz2xyzIdx(Vec3 xyz_w);
    bool xyz2xyzIdxwithBoderCheck(Vec3 xyz_w, Vec3I &xyz_idx);
    size_t mapIdx(Vec3I xyz_idx);
    size_t mapIdx(unsigned int x_idx, unsigned int y_idx, unsigned int z_idx);
    Vec3I global_xyzidx(size_t local_idx);
    Vec3I global_xyzidx(size_t local_idx, Vec3 &pt_w);
    bool inside_exp_bd(Vec3 pt_w);
    void update_observation(size_t map_idx);
    size_t get_subbox_id(Vec3 pt_w);
    size_t get_subbox_id(Vec3 pt_w, Vec3I glb_idx);
    PointP subbox_id2xyz_glb(Vec3I origin, int idx);
};

#endif // local_map_cartesian_H
