#include "map_local.h"

local_map_cartesian::local_map_cartesian()
{
}

void local_map_cartesian::update_observation(Vec3I glb_idx, size_t subbox_id, Vec3 pt_w)
{
    // assume the current cell is free, find the frontiers to the unknown space

    if (!inside_exp_bd(pt_w))
        return; // this subbox is observed completely, no need to update this region.

    observed_subboxes.emplace(glb_idx);
    Mat6x4I nbrs = subbox_neighbors[subbox_id];
    Vec3I glb_idx_nb;
    size_t subbox_id_nb;
    for (auto i = 0; i < 6; i++)
    {
        Vec3 pt_w_nb = pt_w + nbr_disp_real[i];
        glb_idx_nb = glb_idx + Vec3I(nbrs(i, 0), nbrs(i, 1), nbrs(i, 2)); // add the displacement of global idx
        subbox_id_nb = nbrs(i, 3);
        // cout<<"i: "<<i<<" frontier pos: "<<pt_w_nb.transpose()<<" glb_idx_nb: "<<glb_idx_nb.transpose()<<" subbox_id: "<<subbox_id_nb<<endl;
        if (inside_exp_bd(pt_w_nb) && allocate_ram(glb_idx_nb) &&
            // observed_cell_global_idx_map.find(glb_idx) == observed_cell_global_idx_map.end() &&
            (observed_group_map[glb_idx_nb].occupancy[subbox_id_nb] == 'u'))
        {
            observed_group_map[glb_idx_nb].frontier.emplace(subbox_id_nb);
            break;
        }
    }

    // cout << "observe update" << endl;
    return;
}

Vec3I local_map_cartesian::xyz2xyzIdx(Vec3 xyz_w)
{
    double x = xyz_w(0) - map_min_x;
    int x_idx = static_cast<int>(x / map_dxyz);
    double y = xyz_w(1) - map_min_y;
    int y_idx = static_cast<int>(y / map_dxyz);
    double z = xyz_w(2) - map_min_z;
    int z_idx = static_cast<int>(z / map_dxyz);
    return Vec3I(x_idx, y_idx, z_idx);
}


void local_map_cartesian::init_map(double d_xyz_in,
                                   unsigned int subbox_n,
                                   float log_odds_min_in,
                                   float log_odds_max_in,
                                   float log_odds_hit_in,
                                   float log_odds_miss_in,
                                   float log_odds_occupied_sh_in,
                                   bool if_apply_explor)
{
    // map_dxyz = d_xyz_in;
    map_dxyz_obv_sub = d_xyz_in;
    map_dxyz_obv_sub_half = map_dxyz_obv_sub * 0.5;
    map_reso_inv = 1/map_dxyz_obv_sub;
    subbox_nxyz = subbox_n;                                  // subbox size
    map_dxyz_obv_glb = map_dxyz_obv_sub * subbox_nxyz; // temporary
    // const int subbox_nxyz = static_cast<int>(map_dxyz_obv_glb / map_dxyz);
    cell_num_subbox = pow(subbox_nxyz, 3);
    int cnt = 0;
    for (auto i = 0; i < subbox_nxyz; i++) // i-z j-y k-x
    {
        for (auto j = 0; j < subbox_nxyz; j++)
        {
            for (auto k = 0; k < subbox_nxyz; k++)
            {
                subbox_cell_id_table[Vec3I(k, j, i)] = cnt++;
                // cnt is: i * subbox_nxyz * subbox_nxyz + j * subbox_nxyz + k;
                subbox_id2xyz_table.emplace_back(k, j, i);
            }
        }
    }


    Vec3I temp_id;
    nbr_disp.emplace_back(Vec3I(0, 0, 1));
    nbr_disp.emplace_back(Vec3I(0, 0, -1));
    nbr_disp.emplace_back(Vec3I(0, 1, 0));
    nbr_disp.emplace_back(Vec3I(0, -1, 0));
    nbr_disp.emplace_back(Vec3I(1, 0, 0));
    nbr_disp.emplace_back(Vec3I(-1, 0, 0));
    nbr_disp_real.emplace_back(Vec3(0, 0, map_dxyz_obv_sub));
    nbr_disp_real.emplace_back(Vec3(0, 0, -map_dxyz_obv_sub));
    nbr_disp_real.emplace_back(Vec3(0, map_dxyz_obv_sub, 0));
    nbr_disp_real.emplace_back(Vec3(0, -map_dxyz_obv_sub, 0));
    nbr_disp_real.emplace_back(Vec3(map_dxyz_obv_sub, 0, 0));
    nbr_disp_real.emplace_back(Vec3(-map_dxyz_obv_sub, 0, 0));
    for (auto i = 0; i < subbox_nxyz; i++) // get the neighbors after the look-up table is built
    {
        for (auto j = 0; j < subbox_nxyz; j++)
        {
            for (auto k = 0; k < subbox_nxyz; k++)
            {
                Mat6x4I nbrs;
                for (auto n = 0; n < 6; n++)
                {
                    Vec3I glb_disp(0, 0, 0);
                    temp_id = nbr_disp[n] + Vec3I(k, j, i);
                    for (auto m = 0; m < 3; m++)
                    {
                        if (temp_id[m] >= subbox_nxyz)
                        {
                            glb_disp[m] = 1;
                            temp_id[m] = 0;
                        }
                        else if (temp_id[m] < 0)
                        {
                            glb_disp[m] = -1;
                            temp_id[m] = subbox_nxyz - 1;
                        }
                    }
                    nbrs.row(n) << glb_disp[0], glb_disp[1], glb_disp[2], subbox_cell_id_table[temp_id];
                }

                subbox_neighbors.emplace_back(nbrs);
            }
        }
    }

    apply_explored_area = if_apply_explor;

    global_bd = {-30, 30, -30, 30, 0, 5};
    

    log_odds_min = log_odds_min_in;
    log_odds_max = log_odds_max_in;
    log_odds_hit = log_odds_hit_in;
    log_odds_miss = log_odds_miss_in;
    log_odds_occupied_sh = log_odds_occupied_sh_in;



    vis_paras.map_maxz = (0.5 * map_dxyz) + floor(map_nz / 2.0) * map_dxyz;
    vis_paras.map_minz = -(0.5 * map_dxyz) - floor(map_nz / 2.0) * map_dxyz;
    vis_paras.map_size_xy = map_dxyz * map_nxy;
    vis_paras.map_size_z = map_dxyz * map_nz;
    vis_paras.cube_size_xyz = map_dxyz;

    // this->map = std::unique_ptr<vector<CARTESIAN_CELL>>(new vector<CARTESIAN_CELL>());
}



void local_map_cartesian::input_pc_pose_direct(awareness_map_cylindrical *a_map,
                                               map_warehouse *warehouse)
{
    // local_switch = false;
    SE3 T_wa = a_map->T_wa;
    for (auto pair_ : a_map->hit_idx_odds_hashmap)
    {
        Vec3I glb_idx;
        size_t subbox_id;
        Vec3 p_w = T_wa * a_map->map->at(a_map->mapIdx(pair_.first)).center_pt;
        get_global_idx(p_w, glb_idx, subbox_id);
        // size_t map_idx = mapIdx(xyz_idx);
        if (allocate_ram(glb_idx))
        {
            
            if (observed_group_map[glb_idx].log_odds[subbox_id] <= log_odds_max)
            {
                observed_group_map[glb_idx].log_odds[subbox_id] += logit(pair_.second);
                // observed_group_map[glb_idx].log_odds[subbox_id] = log_odds_max;
            }
            // set observerable
            if (observed_group_map[glb_idx].log_odds[subbox_id] > log_odds_occupied_sh && observed_group_map[glb_idx].occupancy[subbox_id] != 'o')
            {
                observed_group_map[glb_idx].occupancy[subbox_id] = 'o';
                observed_group_map[glb_idx].frontier.erase(subbox_id);
                obs_cnt++;
                // occupied_cell_idx_map.emplace(map->at(map_idx).idx);
            }
        }
        // if (apply_explored_area)
        //     update_observation(map_idx);
    }
    for (auto idx : a_map->miss_idx_set)
    {
        Vec3I glb_idx;
        size_t subbox_id;
        Vec3 p_w = T_wa * a_map->map->at(idx).center_pt;
        get_global_idx(p_w, glb_idx, subbox_id);
        // size_t map_idx = mapIdx(xyz_idx);
        // size_t map_idx = mapIdx(xyz_idx);
        // map->at(map_idx).observed = true;
        if (allocate_ram(glb_idx))
        {
            
            if (observed_group_map[glb_idx].log_odds[subbox_id] >= log_odds_min)
            {
                observed_group_map[glb_idx].log_odds[subbox_id] += log_odds_miss; // log_odds_miss is negative, so add it
                // observed_group_map[glb_idx].log_odds[subbox_id] = log_odds_min;
            }
            // set free
            if (observed_group_map[glb_idx].log_odds[subbox_id] < log_odds_occupied_sh && observed_group_map[glb_idx].occupancy[subbox_id] != 'f')
            {
                if (observed_group_map[glb_idx].occupancy[subbox_id] == 'u' && apply_explored_area)
                    update_observation(glb_idx, subbox_id, p_w);
                observed_group_map[glb_idx].occupancy[subbox_id] = 'f';
                observed_group_map[glb_idx].frontier.erase(subbox_id);
                // occupied_cell_idx_map.erase(map->at(map_idx).idx);
            }
        }
        // if (apply_explored_area)
        //     update_observation(map_idx);
    }
    for (auto glb_idx : observed_subboxes)
    {
        if (observed_group_map.find(glb_idx) != observed_group_map.end() &&
            observed_group_map[glb_idx].occupancy.size() > 1 &&
            observed_group_map[glb_idx].frontier.empty())
        // if current subbox is all explored
        {
            // frontier_group_map.erase(glb_idx);
            if (std::adjacent_find(observed_group_map[glb_idx].occupancy.begin(),
                                   observed_group_map[glb_idx].occupancy.end(),
                                   std::not_equal_to<char>()) == observed_group_map[glb_idx].occupancy.end())
            {
                // auto same_elem = observed_group_map[glb_idx][0];
                observed_group_map[glb_idx].occupancy.resize(1);
                observed_group_map[glb_idx].occupancy.shrink_to_fit();
                observed_group_map[glb_idx].log_odds.resize(1);
                observed_group_map[glb_idx].log_odds.shrink_to_fit();
                // observed_group_map.erase(glb_idx);
                cout << "memory release!" << glb_idx.transpose() << endl;
            }
        }
    }
    observed_subboxes.clear();
    cout << "memory expand times: " << ram_expand_cnt << " obs number: " << obs_cnt << endl;
    // cout << "frontier size: " << frontier_cell_global_idx_map.size() << " observed size: " << observed_cell_global_idx_map.size() << endl;
    // cout << occupied_cell_idx_map.size()<<endl;
    // cout << PC_miss_a.size()<<endl;
}