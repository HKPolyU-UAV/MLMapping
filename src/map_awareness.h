#ifndef awareness_map_cylindrical_H
#define awareness_map_cylindrical_H

#include <utils/include/all_utils.h>
#include <mlmapping/awareness2local.h>
#include <data_type.h>

class awareness_map_cylindrical
{
private:
    int nRho_x_nPhi;
    SE3 T_bs; // Transformation from sensor to body
    bool visibility_check;
    double map_dRho;
    double map_dPhi;
    int map_nRho;
    int map_nPhi;
    int map_center_z_idx;
    SE3 last_T_wa;
    bool first_input;
    float sigma_in_dr(size_t x);
    float standard_ND(float x);
    float get_odds(int diff, size_t r);
    void update_hits(Vec3 p_l, Vec3I rpz_idx, size_t map_idx);
    size_t mapIdx(Vec3I Rho_Phi_z);
    size_t mapIdx(int Rho, int Phi, int z);
    double fast_atan2(double y, double x);
    double fast_atan(double x);
    vector<vector<float>> get_odds_table;
    int diff_range;
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
    void update_odds_hashmap(Vec3I rpz_idx, float odd);

public:
    SE3 T_wa;
    int map_nZ;
    double map_dZ;
    double z_border_min;
    // Map
    std::unique_ptr<vector<CYLINDRICAL_CELL>> map;
    std::unique_ptr<vector<CYLINDRICAL_CELL>> map_tmp;
    // local to global messages
    vector<Vec3> l2g_msg_hit_pts_l;  // pts, hit(occupied) measurement, increase the log-odd value
    vector<Vec3> l2g_msg_miss_pts_l; // pts, miss(non-occupied) measurement, decrease the log-odd value
    vector<float> l2g_msg_hit_odds_l;
    unordered_map<Vec3I, float, VectorHasher> hit_idx_odds_hashmap;
    unordered_set<size_t> miss_idx_set;
    // Occupied Idx
    vector<unsigned int> occupied_cell_idx;
    // vector<int> diff_r = {-2, -1, 1, 2};
    // Init
    awareness_map_cylindrical();
    void setTbs(SE3 T_bs_in);
    void init_map(double d_Rho, double d_Phi_deg, double d_Z, int n_Rho, int n_z_below, int n_z_over, bool apply_raycasting);
    void clear_map();
    void creat_transfer_chart();
    // Visit certain cell
    size_t mapIdx_out(Vec3I Rho_Phi_z);

    bool xyz2RhoPhiZwithBoderCheck(Vec3 xyz_l, Vec3I &rhophiz, bool &can_do_cast);

    // input callback
    void input_pc_pose(vector<Vec3> PC_s, SE3 T_wb);
};

#endif // awareness_map_cylindrical_H
