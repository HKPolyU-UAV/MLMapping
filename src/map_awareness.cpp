#include "map_awareness.h"

awareness_map_cylindrical::awareness_map_cylindrical()
{

}

inline size_t awareness_map_cylindrical::mapIdx(Vec3I Rho_Phi_z)
{
    return this->mapIdx(Rho_Phi_z(0),Rho_Phi_z(1),Rho_Phi_z(2));
}
inline size_t awareness_map_cylindrical::mapIdx(int Rho, int Phi, int z)
{
    return static_cast<size_t>(z*this->nRho_x_nPhi
                               +Phi*this->map_nRho
                               +Rho);
}

void awareness_map_cylindrical::setTbs(SE3 T_bs_in)
{
    this->T_bs=T_bs_in;
}

void awareness_map_cylindrical::init_map(double d_Rho, double d_Phi_deg, double d_Z, int n_Rho, int n_z_below, int n_z_over, bool apply_raycasting)
{
    this->map_dRho = d_Rho;
    this->map_dPhi = d_Phi_deg * M_PI / 180;
    this->map_dZ   = d_Z;

    this->map_nRho         = n_Rho;
    this->map_nPhi         = static_cast<int>(360/d_Phi_deg);
    this->map_nZ           = n_z_below+n_z_over+1;
    cout <<  "localmap discretization to n_Rho: " << this->map_nRho  << "  n_Phi:" << this->map_nPhi << "  n_Z:" << this->map_nZ << endl;
    this->map_center_z_idx = n_z_below;
    this->z_border_min     = -(n_z_below*d_Z)-0.5*d_Z;
    this->nRho_x_nPhi = map_nRho*map_nPhi;
    this->map     = std::unique_ptr<vector<CYLINDRICAL_CELL>>(new vector<CYLINDRICAL_CELL>());
    this->map_tmp = std::unique_ptr<vector<CYLINDRICAL_CELL>>(new vector<CYLINDRICAL_CELL>());

    int idx_in_order=0;
    for (int z=0; z<this->map_nZ; z++)
    {
        for (int phi=0; phi<this->map_nPhi; phi++)
        {
            for (int rho=0; rho<this->map_nRho; rho++)
            {
                struct CYLINDRICAL_CELL cell;
                cell.idx_rho = rho;
                cell.idx_phi = phi;
                cell.idx_z = z;
                double center_z =   this->z_border_min+(this->map_dZ/2)+(z*this->map_dZ);
                double center_rho = this->map_dRho/2+(rho*this->map_dRho);
                double center_phi = this->map_dPhi/2+(phi*this->map_dPhi);
                double center_x = center_rho*cos(center_phi);
                double center_y = center_rho*sin(center_phi);
                cell.center_pt = Vec3(center_x,center_y,center_z);
                cell.is_occupied = false;
                if(rho>0){
                    cell.raycasting_z_over_rho = (z-map_center_z_idx)/(rho*1.0);
                }else {
                    cell.raycasting_z_over_rho = 0;
                }
                cell.idx=idx_in_order;
                idx_in_order++;
                this->map->push_back(cell);
                this->map_tmp->push_back(cell);
            }
        }
    }
    cout << "awareness map contain " << map->size() << " cells"<< endl;
    first_input = true;
    visibility_check = apply_raycasting;
}

bool awareness_map_cylindrical::xyz2RhoPhiZwithBoderCheck(Vec3 xyz_l, Vec3I &rhophiz)
{
    double rho = sqrt(pow(xyz_l(0),2)+pow(xyz_l(1),2));
    int rho_idx =  static_cast<int>(rho/this->map_dRho);
    double phi = atan2(xyz_l(1),xyz_l(0));
    if (phi<0) phi += 2*M_PI;
    int phi_idx = static_cast<int>(phi/this->map_dPhi);
    double z = xyz_l(2)-this->z_border_min;
    int z_idx = static_cast<int>(floor(z/this->map_dZ));
    rhophiz = Vec3I(rho_idx,phi_idx,z_idx);
    if(     rho_idx>0
            && phi_idx>0
            && z_idx>0
            && rho_idx < this->map_nRho
            && phi_idx < this->map_nPhi
            && z_idx < this->map_nZ)
    {
        return true;
    }
    return false;
}

void awareness_map_cylindrical::clear_map()
{
    for(auto& cell:*this->map)
    {
        cell.is_occupied = false;
    }
    this->occupied_cell_idx.clear();
}

void awareness_map_cylindrical::input_pc_pose(vector<Vec3> PC_s, SE3 T_wb)
{
    this->l2g_msg_hit_pts_l.clear();
    this->l2g_msg_miss_pts_l.clear();
    this->occupied_cell_idx.clear();

    //STEP 1: transfer from previous map
    //Frame [w]orld, [s]ensor, [b]ody, [l]ocalmap; [g]lobalmap
    T_wa = SE3(SO3(Quaterniond(1,0,0,0)),T_wb.translation());
    SE3 T_ws = T_wb * this->T_bs;
    SE3 T_ls = T_wa.inverse() * T_ws;
    if(first_input)
    {
        first_input = false;
    }else
    {
        //map_tmp = map;
        this->map.swap(this->map_tmp);
        for(auto& cell:*this->map)
        {
            cell.is_occupied = false;
        }
        Vec3 t_diff = -(T_wa.translation()-this->last_T_wa.translation());
        for(auto& cell:*this->map_tmp)
        {
            if(cell.is_occupied)
            {
                Vec3 transfered_pt_l = cell.sampled_xyz + t_diff;
                Vec3I rpz_idx;
                if(xyz2RhoPhiZwithBoderCheck(transfered_pt_l,rpz_idx))
                {//set observerable
                    size_t map_idx = mapIdx(rpz_idx);
                    if(map->at(map_idx).is_occupied == false)
                    {
                        map->at(map_idx).is_occupied = true;
                        map->at(map_idx).sampled_xyz = transfered_pt_l;
                        this->occupied_cell_idx.push_back(map_idx);
                    }
                }
            }
        }
    }
    //STEP 2: Add measurement
    for(auto p_s:PC_s)
    {
        auto p_l = T_ls*p_s;
        Vec3I rpz_idx;
        if(xyz2RhoPhiZwithBoderCheck(p_l,rpz_idx))
        {
            l2g_msg_hit_pts_l.push_back(p_l);
            //set observerable
            size_t map_idx = mapIdx(rpz_idx);
            if(map->at(map_idx).is_occupied == false)
            {
                map->at(map_idx).is_occupied = true;
                map->at(map_idx).sampled_xyz = p_l;
                this->occupied_cell_idx.push_back(map_idx);
            }
            if(visibility_check)
            {
                double raycasting_rate = map->at(map_idx).raycasting_z_over_rho;
                for (int r=rpz_idx[0]-4; r>0 ; r--)
                {
                    int diff_r = rpz_idx[0]-r;
                    int raycasting_z = static_cast<int>(round(rpz_idx[2]-(diff_r*raycasting_rate)));
                    map->at(this->mapIdx(Vec3I(r,rpz_idx[1],raycasting_z))).is_occupied = false;
                    l2g_msg_miss_pts_l.push_back(map->at(this->mapIdx(Vec3I(r,rpz_idx[1],raycasting_z))).center_pt);
                }
            }
        }
    }
    this->last_T_wa = T_wa;
}
