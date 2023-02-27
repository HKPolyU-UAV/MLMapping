#ifndef awareness_map_cylindrical_H
#define awareness_map_cylindrical_H

// #include "all_utils.h"
// #include <mlmapping/awareness2local.h>
#include "data_type.h"
#define deg2rad M_PI / 180
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
    double noise_coe_;
    SE3 last_T_wa;
    bool first_input;
    inline float sigma_in_dr(size_t x);
    inline float standard_ND(float x);
    float get_odds(int diff, size_t r);
    void update_hits(Vec3 p_l, Vec3I rpz_idx, size_t map_idx);

    inline double fast_atan2(double y, double x);
    inline double fast_atan(double x);
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
    inline void update_odds_hashmap(Vec3I rpz_idx, float odd);

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
    inline size_t mapIdx(Vec3I Rho_Phi_z);
    inline size_t mapIdx(int Rho, int Phi, int z);
    void setTbs(SE3 T_bs_in);
    void init_map(double d_Rho, double d_Phi_deg, double d_Z, int n_Rho, int n_z_below, int n_z_over, bool apply_raycasting, double noise_coe);
    void clear_map();
    // Visit certain cell
    // size_t mapIdx_out(Vec3I Rho_Phi_z);

    bool xyz2RhoPhiZwithBoderCheck(Vec3 xyz_l, Vec3I &rhophiz, bool &can_do_cast);

    // input callback
    void input_pc_pose(vector<Vec3> PC_s, SE3 T_wb);
};

inline size_t awareness_map_cylindrical::mapIdx(Vec3I Rho_Phi_z)
{
    return this->mapIdx(Rho_Phi_z(0), Rho_Phi_z(1), Rho_Phi_z(2));
}
inline size_t awareness_map_cylindrical::mapIdx(int Rho, int Phi, int z)
{
    return static_cast<size_t>(z * this->nRho_x_nPhi + Phi * this->map_nRho + Rho);
}

inline double awareness_map_cylindrical::fast_atan2(double y, double x)
{
    // 1. map the input to 0-1
    double input = y / x;
    double a_input = abs(input);
    double res;
    //   int sign = a_input/input;
    if (a_input > 1)
    {
        res = copysign(deg2rad * (90 - fast_atan(1 / a_input)), input);
    }
    else
    {
        res = copysign(deg2rad * fast_atan(a_input), input);
    }
    if (x > 0)
    {
        return res;
    }
    else if (y >= 0)
    {
        return res + M_PI;
    }
    else
    {
        return res - M_PI;
    }
}

inline double awareness_map_cylindrical::fast_atan(double x)
{
    return x * (45 - (x - 1) * (14 + 3.83 * x));
}

inline float awareness_map_cylindrical::sigma_in_dr(size_t x)
{
    float dis = (x * this->map_dRho);
    return noise_coe_ * dis * dis / this->map_dRho; // for realsense d435i
}

inline float awareness_map_cylindrical::standard_ND(float x)
{
    double a1 = 0.254829592;
    double a2 = -0.284496736;
    double a3 = 1.421413741;
    double a4 = -1.453152027;
    double a5 = 1.061405429;
    double p = 0.3275911;

    // Save the sign of x
    int sign = 1;
    if (x < 0)
        sign = -1;
    x = fabs(x) / sqrt(2.0);

    // A&S formula 7.1.26
    double t = 1.0 / (1.0 + p * x);
    double y = 1.0 - (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * exp(-x * x);
    // cout<<"y: "<<y<<" x: "<<x<<" exp: "<<exp(-x * x)<<endl;
    return 0.5 * (1.0 + sign * y); // a close representation for the cumulative density function of a standard normal distribution, see http://www.johndcook.com/blog/cpp_phi/
}
inline void awareness_map_cylindrical::update_odds_hashmap(Vec3I rpz_idx, float odd)
{
    if (hit_idx_odds_hashmap.find(rpz_idx) == hit_idx_odds_hashmap.end())

        hit_idx_odds_hashmap[rpz_idx] = odd;
    else
        hit_idx_odds_hashmap[rpz_idx] = 1 - (1 - hit_idx_odds_hashmap[rpz_idx]) * (1 - odd);
}
#endif // awareness_map_cylindrical_H
