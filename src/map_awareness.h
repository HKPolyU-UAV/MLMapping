#ifndef awareness_map_cylindrical_H
#define awareness_map_cylindrical_H

#include <utils/include/all_utils.h>
#include <mlmapping/awareness2local.h>
#include <data_type.h>


class awareness_map_cylindrical
{
private:
    int nRho_x_nPhi;
    SE3 T_bs; //Transformation from sensor to body
    bool visibility_check;
    double map_dRho;
    double map_dPhi;
    int map_nRho;
    int map_nPhi;
    int map_center_z_idx;
    SE3 last_T_wa;
    bool first_input;

public:
    SE3 T_wa;
    int map_nZ;
    double map_dZ;
    double z_border_min;
    //Map
    std::unique_ptr<vector<CYLINDRICAL_CELL>> map;
    std::unique_ptr<vector<CYLINDRICAL_CELL>> map_tmp;
    //local to global messages
    vector<Vec3> l2g_msg_hit_pts_l; //pts, hit(occupied) measurement, increase the log-odd value
    vector<Vec3> l2g_msg_miss_pts_l;//pts, miss(non-occupied) measurement, decrease the log-odd value
    //Occupied Idx
    vector<unsigned int> occupied_cell_idx;

    //Init
    awareness_map_cylindrical();
    void setTbs(SE3 T_bs_in);
    void init_map(double d_Rho, double d_Phi_deg, double d_Z, int n_Rho, int n_z_below, int n_z_over, bool apply_raycasting);
    void clear_map();
    void creat_transfer_chart();
    //Visit certain cell
    size_t mapIdx(Vec3I Rho_Phi_z);
    size_t mapIdx(int Rho, int Phi, int z);
    bool   xyz2RhoPhiZwithBoderCheck(Vec3 xyz_l, Vec3I &rhophiz);
    //input callback
    void input_pc_pose(vector<Vec3> PC_s, SE3 T_wb);
};

#endif // awareness_map_cylindrical_H
