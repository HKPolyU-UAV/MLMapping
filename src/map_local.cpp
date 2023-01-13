#include "map_local.h"

local_map_cartesian::local_map_cartesian()
{
}

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

inline size_t local_map_cartesian::get_subbox_id(Vec3 pt_w, Vec3I glb_idx)
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

PointP local_map_cartesian::subbox_id2xyz_glb(Vec3I origin, int idx)
{
    return PointP(origin[0] * map_dxyz_obv_glb + subbox_id2xyz_table[idx][0] * map_dxyz_obv_sub,
                  origin[1] * map_dxyz_obv_glb + subbox_id2xyz_table[idx][1] * map_dxyz_obv_sub,
                  origin[2] * map_dxyz_obv_glb + subbox_id2xyz_table[idx][2] * map_dxyz_obv_sub);
}

void local_map_cartesian::update_observation(size_t map_idx)
{
    // map->at(map_idx).observed_free = true;
    Vec3 pt_w;
    size_t subbox_id;
    Vec3I glb_idx = global_xyzidx(map_idx, pt_w);
    if (!inside_exp_bd(pt_w) | (observed_group_map.find(glb_idx) != observed_group_map.end() && observed_group_map[glb_idx].empty()))
        return; // this subbox is observed completely, no need to update this region.
    observed_subboxes.emplace(glb_idx);
    // PointP p1 = subbox_id2xyz_glb(glb_idx,2);
    // observed_cell_global_idx_map[glb_idx] = true;
    // frontier_cell_global_idx_map.erase(glb_idx);

    if (observed_group_map.find(glb_idx) == observed_group_map.end()) // if current point belongs to a new group
    {
        observed_group_map[glb_idx].resize(cell_num_subbox, false);
        frontier_group_map[glb_idx].clear();
        ram_expand_cnt++;
    }
    else
    {
        subbox_id = get_subbox_id(pt_w, glb_idx);
        if (!observed_group_map[glb_idx][subbox_id])
        {
            observed_group_map[glb_idx][subbox_id] = true;
            frontier_group_map[glb_idx].erase(subbox_id);
        }
        else
            return; // this cell is observed before, so no need to update any more
    }
    // if (map->at(map_idx).is_occupied)
    for (auto i = 0; i < 6; i++)
    {
        size_t nb = map->at(map_idx).neighbors[i];
        glb_idx = global_xyzidx(nb, pt_w);
        subbox_id = get_subbox_id(pt_w, glb_idx);
        if (observed_group_map.find(glb_idx) == observed_group_map.end()) // if current point belongs to a new group
        {
            observed_group_map[glb_idx].resize(cell_num_subbox, false);
            frontier_group_map[glb_idx].clear();
            ram_expand_cnt++;
        }
        if (inside_exp_bd(pt_w) &&
            // observed_cell_global_idx_map.find(glb_idx) == observed_cell_global_idx_map.end() &&
            (!observed_group_map[glb_idx][subbox_id]) &&
            !map->at(nb).is_occupied)
        // inside the exploration bound, neighbor cell is not observed and is not obstacle (then must be unknown)
        {
            // frontier_cell_global_idx_map[glb_idx] = true;
            frontier_group_map[glb_idx].emplace(subbox_id);
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

void local_map_cartesian::devide_local_map_to_submaps()
{
    Vec3 diff_min_center = map_min_xyz - map_center_xyz;
    for (int i = 0; i < 125; i++)
    {
        sub_maps[i].cells.clear();
        sub_maps[i].submap_info.center_xyz = Vec3(
            map_min_x + submap_paras.submap_diff_center_offset(0) + (i % 5) * map_dxyz * submap_paras.submap_nxy,
            map_min_y + submap_paras.submap_diff_center_offset(1) + ((i % 25) / 5) * map_dxyz * submap_paras.submap_nxy,
            map_min_z + submap_paras.submap_diff_center_offset(2) + (i / 25) * map_dxyz * submap_paras.submap_nz);
        sub_maps[i].submap_info.offset_min_xyz = diff_min_center + sub_maps[i].submap_info.center_xyz;
        // cout<<sizeof(sub_map_switching_check_list)<<endl;
        // cout << "submap idx: " << i << " center: " << sub_map_switching_check_list[i].submap_info.center_xyz.transpose() << endl;
    }
    for (auto iter = occupied_cell_idx_map.begin(); iter != occupied_cell_idx_map.end(); ++iter)
    {
        CARTESIAN_CELL cell_in_localmap = this->map->at(iter->first);
        SUBMAP_CELL cell_in_submap;
        cell_in_submap.pt_w = cell_in_localmap.center_pt + this->map_center_xyz;
        cell_in_submap.log_odds = cell_in_localmap.log_odds;
        sub_maps[cell_in_localmap.relevant_submap_idx].cells.push_back(cell_in_submap);
        // clear the oringinal local map
        this->map->at(iter->first).log_odds = 0;
        this->map->at(iter->first).is_occupied = false;
    }
    this->occupied_cell_idx_map.clear();
    // for (auto iter = frontier_cell_idx_map.begin(); iter != frontier_cell_idx_map.end(); ++iter)
    // {
    //     CARTESIAN_CELL cell_in_localmap = this->map->at(iter->first);
    //     SUBMAP_CELL cell_in_submap;
    //     cell_in_submap.pt_w = cell_in_localmap.center_pt + this->map_center_xyz;
    //     cell_in_submap.
    //     sub_maps[cell_in_localmap.relevant_submap_idx].cells.push_back(cell_in_submap);
    //     // clear the oringinal local map
    //     this->map->at(iter->first).is_occupied = false;
    // }
    // this->frontier_cell_idx_map.clear();
}

void local_map_cartesian::init_map(double d_xyz_in,
                                   unsigned int n_xy_in,
                                   unsigned n_z_in,
                                   float log_odds_min_in,
                                   float log_odds_max_in,
                                   float log_odds_hit_in,
                                   float log_odds_miss_in,
                                   float log_odds_occupied_sh_in,
                                   bool if_apply_explor)
{
    map_dxyz = d_xyz_in;
    map_dxyz_obv_sub = 0.5;
    subbox_nxyz = 10;                                  // subbox size
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
    apply_explored_area = if_apply_explor;
    map_nxy = n_xy_in;
    map_nz = n_z_in;
    submap_paras.submap_nxy = n_xy_in / 5;
    submap_paras.submap_nz = n_z_in / 5;
    global_bd = {-30, 30, -30, 30, 0, 4};
    // global_min_idxx = static_cast<int>(global_bd[0] / map_dxyz_obv_glb);
    // global_min_idxy = static_cast<int>(global_bd[2] / map_dxyz_obv_glb);
    // global_min_idxz = static_cast<int>(global_bd[4] / map_dxyz_obv_glb);
    // global_max_idxx = static_cast<int>(global_bd[1] / map_dxyz_obv_glb);
    // global_max_idxy = static_cast<int>(global_bd[3] / map_dxyz_obv_glb);
    // global_max_idxz = static_cast<int>(global_bd[5] / map_dxyz_obv_glb);
    if ((map_nxy != submap_paras.submap_nxy * 5) || (map_nz != submap_paras.submap_nz * 5))
    {
        cout << "critical error"
             << "please check the config file and make sure nxyz=(2n+1)*5" << endl;
        while (1)
            ;
    }
    submap_paras.submap_diff_center_offset = Vec3(
        (0.5 * map_dxyz) + floor(submap_paras.submap_nxy / 2.0) * map_dxyz,
        (0.5 * map_dxyz) + floor(submap_paras.submap_nxy / 2.0) * map_dxyz,
        (0.5 * map_dxyz) + floor(submap_paras.submap_nz / 2.0) * map_dxyz);
    submap_paras.diff_1st_close_submap_center_localmap_min = Vec3(
        (0.5 * map_dxyz) + (floor(submap_paras.submap_nxy / 2.0) + submap_paras.submap_nxy) * map_dxyz,
        (0.5 * map_dxyz) + (floor(submap_paras.submap_nxy / 2.0) + submap_paras.submap_nxy) * map_dxyz,
        (0.5 * map_dxyz) + (floor(submap_paras.submap_nz / 2.0) + submap_paras.submap_nz) * map_dxyz);
    submap_paras.relevant_submap_search_range_xy = 2.2 * this->submap_paras.submap_nxy * this->map_dxyz;
    submap_paras.relevant_submap_search_range_z = 2.2 * this->submap_paras.submap_nz * this->map_dxyz;
    map_nx_times_ny = n_xy_in * n_xy_in;
    map_min_x = 0;
    map_min_y = 0;
    map_min_z = 0;
    first_pose = true;

    log_odds_min = log_odds_min_in;
    log_odds_max = log_odds_max_in;
    log_odds_hit = log_odds_hit_in;
    log_odds_miss = log_odds_miss_in;
    log_odds_occupied_sh = log_odds_occupied_sh_in;

    for (size_t i = 0; i < 27; i++)
    {
        sub_map_switching_check_list[i].local_sub_map_idx = static_cast<unsigned int>(i);
    }
    unique_submap_idx = 0;
    occupied_cell_idx_map.clear();

    vis_paras.map_maxz = (0.5 * map_dxyz) + floor(map_nz / 2.0) * map_dxyz;
    vis_paras.map_minz = -(0.5 * map_dxyz) - floor(map_nz / 2.0) * map_dxyz;
    vis_paras.map_size_xy = map_dxyz * map_nxy;
    vis_paras.map_size_z = map_dxyz * map_nz;
    vis_paras.cube_size_xyz = map_dxyz;

    this->map = std::unique_ptr<vector<CARTESIAN_CELL>>(new vector<CARTESIAN_CELL>());
}

void local_map_cartesian::clear_map()
{
    for (auto &cell : *this->map)
    {
        cell.is_occupied = false;
        cell.log_odds = 0;
    }
    this->occupied_cell_idx_map.clear();
}

void local_map_cartesian::allocate_memory_for_local_map()
{
    unsigned int idx_in_order = 0;
    double minxy, minz;
    minxy = -(0.5 * map_dxyz) - floor(map_nxy / 2.0) * map_dxyz;
    minz = -(0.5 * map_dxyz) - floor(map_nz / 2.0) * map_dxyz;
    for (unsigned int z = 0; z < map_nz; z++)
    {
        for (unsigned int y = 0; y < map_nxy; y++)
        {
            for (unsigned int x = 0; x < map_nxy; x++)
            {
                CARTESIAN_CELL cell;
                cell.is_occupied = false;
                // cell.observed = false;
                cell.sd = 0;
                cell.idx_x = x;
                cell.idx_y = y;
                cell.idx_z = z;
                cell.project2d_idx = x + y * map_nxy;
                cell.log_odds = 0;
                cell.idx = idx_in_order;
                cell.center_pt = Vec3(minxy + (map_dxyz / 2) + (x * map_dxyz),
                                      minxy + (map_dxyz / 2) + (y * map_dxyz),
                                      minz + (map_dxyz / 2) + (z * map_dxyz));
                cell.relevant_submap_idx = (z / submap_paras.submap_nz) * 25 + (y / submap_paras.submap_nxy) * 5 + (x / submap_paras.submap_nxy);
                if (z < map_nz - 1 && z > 0 && y < map_nxy - 1 && y > 0 && x < map_nxy - 1 && x > 0)
                {
                    cell.neighbors[0] = mapIdx(x, y, z + 1);
                    cell.neighbors[1] = mapIdx(x, y, z - 1);
                    cell.neighbors[2] = mapIdx(x, y + 1, z);
                    cell.neighbors[3] = mapIdx(x, y - 1, z);
                    cell.neighbors[4] = mapIdx(x + 1, y, z);
                    cell.neighbors[5] = mapIdx(x - 1, y, z);
                }
                map->push_back(cell);
                idx_in_order++;
            }
        }
    }
    cout << "local map contain " << map->size() << " cells" << endl;
}

bool local_map_cartesian::get_the_relevant_submap_for_new_localmap(vector<unsigned int> &relevant_and_occupied_idx,
                                                                   vector<unsigned int> &unrelevant_and_occupied_idx)
{
    relevant_and_occupied_idx.clear();
    unrelevant_and_occupied_idx.clear();
    bool ret = false;
    for (unsigned int i = 0; i < 125; i++)
    {
        if (this->sub_maps[i].cells.size() == 0)
            continue;
        Vec3 diff = this->sub_maps[i].submap_info.center_xyz - this->map_center_xyz;
        if (fabs(diff(0)) < submap_paras.relevant_submap_search_range_xy &&
            fabs(diff(1)) < submap_paras.relevant_submap_search_range_xy &&
            fabs(diff(2)) < submap_paras.relevant_submap_search_range_z)
        {
            relevant_and_occupied_idx.push_back(i);
            ret = true;
        }
        else
        {
            unrelevant_and_occupied_idx.push_back(i);
        }
    }
    return ret;
}

void local_map_cartesian::input_pc_pose(vector<Vec3> PC_hit_a,
                                        vector<float> PC_odds_a,
                                        vector<Vec3> PC_miss_a,
                                        SE3 T_wa,
                                        map_warehouse *warehouse)
{
    T_wa_latest = T_wa;
    local_switch = false;
    if (first_pose)
    {
        // center_xyz and min_xyz
        double c_x = round(T_wa.translation().x() / map_dxyz) * map_dxyz;
        double c_y = round(T_wa.translation().y() / map_dxyz) * map_dxyz;
        double c_z = round(T_wa.translation().z() / map_dxyz) * map_dxyz;
        map_center_xyz = Vec3(c_x, c_y, c_z);
        map_min_x = map_center_xyz(0) - (0.5 * map_dxyz) - floor(map_nxy / 2.0) * map_dxyz;
        map_min_y = map_center_xyz(1) - (0.5 * map_dxyz) - floor(map_nxy / 2.0) * map_dxyz;
        map_min_z = map_center_xyz(2) - (0.5 * map_dxyz) - floor(map_nz / 2.0) * map_dxyz;
        map_min_xyz = Vec3(map_min_x, map_min_y, map_min_z);
        // T_wl =  SE3(SO3(0,0,0),map_center_xyz);
        // init the switching check list
        Vec3 diff_min_center = map_min_xyz - map_center_xyz;
        // cout << "map center: " << T_wl.translation().transpose() <<  endl;

        for (size_t i = 0; i < 27; i++)
        {
            sub_map_switching_check_list[i].submap_info.center_xyz = Vec3(
                map_min_x + submap_paras.diff_1st_close_submap_center_localmap_min(0) + (i % 3) * map_dxyz * submap_paras.submap_nxy,
                map_min_y + submap_paras.diff_1st_close_submap_center_localmap_min(1) + ((i % 9) / 3) * map_dxyz * submap_paras.submap_nxy,
                map_min_z + submap_paras.diff_1st_close_submap_center_localmap_min(2) + (i / 9) * map_dxyz * submap_paras.submap_nz);
            sub_map_switching_check_list[i].submap_info.offset_min_xyz = diff_min_center + sub_map_switching_check_list[i].submap_info.center_xyz;
            // cout << "submap idx: " << i << " center: " << sub_map_switching_check_list[i].submap_info.center_xyz.transpose() << endl;
        }
        first_pose = false;
    }
    else
    {
        // check whether need to swith to another localmap
        local_switch = true;
        Vec3 t_wa = T_wa.translation();
        double min_distance_2_center = (t_wa - map_center_xyz).norm();
        double min_distance_2_other = 999.0;
        unsigned int min_distance_2_other_idx = 27;
        for (size_t i = 0; i < 27; i++)
        {
            if (i == 13)
                continue;
            double dis = (sub_map_switching_check_list[i].submap_info.center_xyz - t_wa).norm();
            if (dis < min_distance_2_other)
            {
                min_distance_2_other = dis;
                min_distance_2_other_idx = static_cast<unsigned int>(i);
            }
        }
        // if so, switch
        if (min_distance_2_center > min_distance_2_other)
        {
            //            cout << "min_distance_2_center=" << min_distance_2_center << " while " << "min_distance_2_other=" << min_distance_2_other << endl;
            cout << "Local map switch to " << min_distance_2_other_idx << endl;
            // STEP1: devided the map to submaps
            this->devide_local_map_to_submaps();
            // STEP2: switching
            map_center_xyz = sub_map_switching_check_list[min_distance_2_other_idx].submap_info.center_xyz;
            map_min_xyz = sub_map_switching_check_list[min_distance_2_other_idx].submap_info.offset_min_xyz;
            map_min_x = map_min_xyz(0);
            map_min_y = map_min_xyz(1);
            map_min_z = map_min_xyz(2);
            // T_wl =  SE3(SO3(0,0,0),map_center_xyz);
            // cout << "new map center: " << T_wl.translation().transpose() << endl;
            // for(auto& i: *map)
            // {
            //     i.is_occupied = false;
            // }
            // this->clear_map();
            // this->occupied_cell_idx_map.clear();
            // STEP3: recover from warehouse and submaps of previous localmap
            // recover from previous localmap
            vector<unsigned int> relevant, unrelavant, relevant_from_warehouse;
            if (this->get_the_relevant_submap_for_new_localmap(relevant, unrelavant))
            {
                //                cout << "these submap are relevant to the new localmap: ";
                //                for(unsigned int idx:relevant){
                //                    cout << idx << " ";
                //                }
                //                cout << endl;
            }
            for (unsigned int idx : relevant)
            {
                for (SUBMAP_CELL cell : sub_maps[idx].cells)
                {
                    Vec3I xyz_idx;
                    if (xyz2xyzIdxwithBoderCheck(cell.pt_w, xyz_idx))
                    {
                        size_t map_idx = mapIdx(xyz_idx);
                        map->at(map_idx).log_odds = cell.log_odds > log_odds_max ? log_odds_max : cell.log_odds;
                        // cout<<"recover node odds: "<<cell.log_odds<<endl;
                        map->at(map_idx).is_occupied = true;
                        occupied_cell_idx_map[map->at(map_idx).idx] = true;
                    }
                }
            }
            // recover from warehouse
            if (warehouse->searchSubMap(map_center_xyz,
                                        submap_paras.relevant_submap_search_range_xy,
                                        submap_paras.relevant_submap_search_range_z,
                                        relevant_from_warehouse))
            {
                //                cout << "these submap from warehouse are relevant to the new localmap: ";
                //                for(unsigned int idx:relevant_from_warehouse){
                //                    cout << idx << " ";
                //                }
                //                cout << endl;
            }

            // recover from warehouse localmap
            sort(relevant_from_warehouse.begin(), relevant_from_warehouse.end());
            for (unsigned int i = 0; i < warehouse->warehouse.size(); i++)
            {
                if (relevant_from_warehouse.size() == 0)
                    continue;
                if (warehouse->warehouse[i].submap_info.idx == relevant_from_warehouse.front())
                {
                    relevant_from_warehouse.erase(relevant_from_warehouse.begin());
                    for (SUBMAP_CELL cell : warehouse->warehouse[i].cells)
                    {
                        Vec3I xyz_idx;
                        if (xyz2xyzIdxwithBoderCheck(cell.pt_w, xyz_idx))
                        {
                            size_t map_idx = mapIdx(xyz_idx);
                            map->at(map_idx).log_odds = cell.log_odds > log_odds_max ? log_odds_max : cell.log_odds;
                            map->at(map_idx).is_occupied = true;
                            occupied_cell_idx_map[map->at(map_idx).idx] = true;
                        }
                    }
                }
            }
            for (unsigned int idx : relevant)
            {
                for (SUBMAP_CELL cell : sub_maps[idx].cells)
                {
                    Vec3I xyz_idx;
                    if (xyz2xyzIdxwithBoderCheck(cell.pt_w, xyz_idx))
                    {
                        size_t map_idx = mapIdx(xyz_idx);
                        map->at(map_idx).log_odds = cell.log_odds > log_odds_max ? log_odds_max : cell.log_odds;
                        map->at(map_idx).is_occupied = true;
                        occupied_cell_idx_map[map->at(map_idx).idx] = true;
                    }
                }
            }

            // STEP4: init the switching check list
            Vec3 diff_min_center = map_min_xyz - map_center_xyz;
            for (size_t i = 0; i < 27; i++)
            {
                sub_map_switching_check_list[i].submap_info.center_xyz = Vec3(
                    map_min_x + submap_paras.diff_1st_close_submap_center_localmap_min(0) + (i % 3) * map_dxyz * submap_paras.submap_nxy,
                    map_min_y + submap_paras.diff_1st_close_submap_center_localmap_min(1) + ((i % 9) / 3) * map_dxyz * submap_paras.submap_nxy,
                    map_min_z + submap_paras.diff_1st_close_submap_center_localmap_min(2) + (i / 9) * map_dxyz * submap_paras.submap_nz);
                sub_map_switching_check_list[i].submap_info.offset_min_xyz = diff_min_center + sub_map_switching_check_list[i].submap_info.center_xyz;
            }

            // STEP5: add submaps to warehouse & delete submaps in warehouse
            for (unsigned int i = 0; i < unrelavant.size(); i++)
            {
                warehouse->addSubMap(sub_maps[unrelavant.at(i)]);
            }
            warehouse->deleteSubMap(relevant_from_warehouse);
        }
    }
    // update map.

    // Frame [w]orld, [s]ensor, [b]ody, [a]wareness, [l]ocalmap;
    //  vector<Vec3> pc_hit_w;
    //  vector<Vec3> pc_miss_w;
    //  for(auto p_a:PC_hit_a)
    //  {
    //      pc_hit_w.push_back(T_wa*p_a);
    //  }
    //  for(auto p_a:PC_miss_a)
    //  {
    //      pc_miss_w.push_back(T_wa*p_a);
    //  }
    // STEP 2: Add measurement
    int i = 0;
    for (auto p_a : PC_hit_a)
    {
        Vec3I xyz_idx;
        if (xyz2xyzIdxwithBoderCheck(T_wa * p_a, xyz_idx))
        {
            size_t map_idx = mapIdx(xyz_idx);
            // if (local_switch && !map->at(map_idx).is_occupied)
            // map->at(map_idx).log_odds = 0;
            // map->at(map_idx).observed = true;
            // map->at(map_idx).log_odds += log_odds_hit;
            map->at(map_idx).log_odds += PC_odds_a[i++];
            if (map->at(map_idx).log_odds > log_odds_max)
            {
                map->at(map_idx).log_odds = log_odds_max;
            }
            // set observerable
            if (map->at(map_idx).log_odds > log_odds_occupied_sh)
            {
                map->at(map_idx).is_occupied = true;
                occupied_cell_idx_map[map->at(map_idx).idx] = true;
            }
        }
        else
        {
        }
    }
    for (auto p_miss_a : PC_miss_a)
    {
        Vec3I xyz_idx;
        if (xyz2xyzIdxwithBoderCheck(T_wa * p_miss_a, xyz_idx))
        {
            size_t map_idx = mapIdx(xyz_idx);
            // map->at(map_idx).observed = true;
            if (apply_explored_area)
                update_observation(map_idx);
            map->at(map_idx).log_odds += log_odds_miss; // log_odds_miss is negative, so add it
            if (map->at(map_idx).log_odds < log_odds_min)
            {
                map->at(map_idx).log_odds = log_odds_min;
            }
            // set free
            if (map->at(map_idx).log_odds < log_odds_occupied_sh)
            {
                map->at(map_idx).is_occupied = false;
                occupied_cell_idx_map.erase(map->at(map_idx).idx);
            }
        }
        else
        {
        }
    }
    for (auto glb_idx : observed_subboxes)
    {
        if (observed_group_map.find(glb_idx) != observed_group_map.end() &&
            !observed_group_map[glb_idx].empty() &&
            frontier_group_map.find(glb_idx) != frontier_group_map.end() &&
            frontier_group_map[glb_idx].empty())
        // if current subbox is all explored
        {
            // frontier_group_map.erase(glb_idx);
            observed_group_map[glb_idx].clear();
            observed_group_map[glb_idx].shrink_to_fit();
            // observed_group_map.erase(glb_idx);
            cout << "memory release!" << glb_idx.transpose() << endl;
        }
    }
    observed_subboxes.clear();
    cout << "memory expand times: " << ram_expand_cnt << endl;
    // cout << "frontier size: " << frontier_cell_global_idx_map.size() << " observed size: " << observed_cell_global_idx_map.size() << endl;
    // cout << occupied_cell_idx_map.size()<<endl;
    // cout << PC_miss_a.size()<<endl;
    // Update occupied list;
    // occupied_cell_idx.clear();
    // for(auto cell: *map)
    // {
    //     if(cell.is_occupied)
    //     {
    //         occupied_cell_idx.push_back(cell.idx);

    //     }
    // }
    // cout << "globalmap vis size" << visualization_cell_list.size() << endl;
}
