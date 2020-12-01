#include "map_local.h"

local_map_cartesian::local_map_cartesian()
{

}

size_t local_map_cartesian::mapIdx(Vec3I xyz_idx)
{
    return mapIdx(static_cast<unsigned int>(xyz_idx(0)),
                  static_cast<unsigned int>(xyz_idx(1)),
                  static_cast<unsigned int>(xyz_idx(2)));
}
size_t local_map_cartesian::mapIdx(unsigned int x_idx, unsigned int y_idx, unsigned int z_idx)
{
    return static_cast<size_t>( z_idx*map_nx_times_ny
                                +y_idx*map_nxy
                                +x_idx);
}

Vec3I local_map_cartesian::xyz2xyzIdx(Vec3 xyz_w)
{
    double x = xyz_w(0)-map_min_x;
    int x_idx =  static_cast<int>(x/map_dxyz);
    double y = xyz_w(1)-map_min_y;
    int y_idx =  static_cast<int>(y/map_dxyz);
    double z = xyz_w(2)-map_min_z;
    int z_idx =  static_cast<int>(z/map_dxyz);
    return Vec3I(x_idx,y_idx,z_idx);
}

bool local_map_cartesian::xyz2xyzIdxwithBoderCheck(Vec3 xyz_w, Vec3I &xyz_idx)
{
    double x = xyz_w(0)-map_min_x;
    int x_idx =  static_cast<int>(x/map_dxyz);
    double y = xyz_w(1)-map_min_y;
    int y_idx =  static_cast<int>(y/map_dxyz);
    double z = xyz_w(2)-map_min_z;
    int z_idx =  static_cast<int>(z/map_dxyz);
    if(x>0 && y>0 && z>0)
    {
        if( x>0 && y>0 && z>0
                && x_idx>=0 && y_idx>=0 && z_idx>=0
                && x_idx < static_cast<int>(map_nxy)
                && y_idx < static_cast<int>(map_nxy)
                && z_idx < static_cast<int>(map_nz))
        {
            xyz_idx = Vec3I(x_idx,y_idx,z_idx);
            return true;
        }
    }
    return false;
}


void local_map_cartesian::devide_local_map_to_submaps()
{
    Vec3 diff_min_center = map_min_xyz - map_center_xyz;
    for(int i=0;i<125;i++)
    {
        sub_maps[i].cells.clear();
        sub_maps[i].submap_info.center_xyz=Vec3(
                    map_min_x + submap_paras.submap_diff_center_offset(0) + (i%5)*map_dxyz*submap_paras.submap_nxy,
                    map_min_y + submap_paras.submap_diff_center_offset(1) + ((i%25)/5)*map_dxyz*submap_paras.submap_nxy,
                    map_min_z + submap_paras.submap_diff_center_offset(2) + (i/25)*map_dxyz*submap_paras.submap_nz
                    );
        sub_maps[i].submap_info.offset_min_xyz = diff_min_center + sub_map_switching_check_list[i].submap_info.center_xyz;
        //cout << "submap idx: " << i << " center: " << sub_maps[i].submap_info.center_xyz.transpose() << endl;
    }
    for(auto idx:this->occupied_cell_idx)
    {
        CARTESIAN_CELL cell_in_localmap = this->map->at(idx);
        SUBMAP_CELL cell_in_submap;
        cell_in_submap.pt_w = cell_in_localmap.center_pt + this->map_center_xyz;
        cell_in_submap.log_odds = cell_in_localmap.log_odds;
        sub_maps[cell_in_localmap.relevant_submap_idx].cells.push_back(cell_in_submap);
    }
}

void local_map_cartesian::init_map(double d_xyz_in,
                                   unsigned int n_xy_in,
                                   unsigned n_z_in,
                                   float log_odds_min_in,
                                   float log_odds_max_in,
                                   float log_odds_hit_in,
                                   float log_odds_miss_in,
                                   float log_odds_occupied_sh_in)
{
    map_dxyz = d_xyz_in;
    map_nxy = n_xy_in;
    map_nz = n_z_in;
    submap_paras.submap_nxy = n_xy_in/5;
    submap_paras.submap_nz = n_z_in/5;
    if((map_nxy!=submap_paras.submap_nxy*5)||(map_nz!=submap_paras.submap_nz*5))
    {
        cout << "critical error" << "please check the config file and make sure nxyz=(2n+1)*5" << endl;
        while(1);
    }
    submap_paras.submap_diff_center_offset = Vec3(
                (0.5*map_dxyz)+floor(submap_paras.submap_nxy/2.0)*map_dxyz,
                (0.5*map_dxyz)+floor(submap_paras.submap_nxy/2.0)*map_dxyz,
                (0.5*map_dxyz)+floor(submap_paras.submap_nz/2.0)*map_dxyz);
    submap_paras.diff_1st_close_submap_center_localmap_min = Vec3(
                (0.5*map_dxyz)+(floor(submap_paras.submap_nxy/2.0)+submap_paras.submap_nxy)*map_dxyz,
                (0.5*map_dxyz)+(floor(submap_paras.submap_nxy/2.0)+submap_paras.submap_nxy)*map_dxyz,
                (0.5*map_dxyz)+(floor(submap_paras.submap_nz/2.0)+submap_paras.submap_nz)*map_dxyz);
    submap_paras.relevant_submap_search_range_xy = 2.2*this->submap_paras.submap_nxy*this->map_dxyz;
    submap_paras.relevant_submap_search_range_z  = 2.2*this->submap_paras.submap_nz*this->map_dxyz;
    map_nx_times_ny = n_xy_in*n_xy_in;
    map_min_x = 0;
    map_min_y = 0;
    map_min_z = 0;
    first_pose = true;

    log_odds_min = log_odds_min_in;
    log_odds_max = log_odds_max_in;
    log_odds_hit = log_odds_hit_in;
    log_odds_miss = log_odds_miss_in;
    log_odds_occupied_sh = log_odds_occupied_sh_in;

    for(size_t i=0; i<27; i++)
    {
        sub_map_switching_check_list[i].local_sub_map_idx=static_cast<unsigned int>(i);
    }
    unique_submap_idx = 0;
    occupied_cell_idx.clear();

    vis_paras.map_maxz = (0.5*map_dxyz)+floor(map_nz/2.0)*map_dxyz;
    vis_paras.map_minz = -(0.5*map_dxyz)-floor(map_nz/2.0)*map_dxyz;
    vis_paras.map_size_xy = map_dxyz*map_nxy;
    vis_paras.map_size_z = map_dxyz*map_nz;
    vis_paras.cube_size_xyz = map_dxyz;

    this->map = std::unique_ptr<vector<CARTESIAN_CELL>>(new vector<CARTESIAN_CELL>());

}

void local_map_cartesian::clear_map()
{
    for(auto& cell: *this->map)
    {
        cell.is_occupied = false;
    }
    this->occupied_cell_idx.clear();
}

void local_map_cartesian::allocate_memory_for_local_map()
{
    unsigned int idx_in_order=0;
    double minxy,minz;
    minxy = - (0.5*map_dxyz) - floor(map_nxy/2.0)*map_dxyz;
    minz  = - (0.5*map_dxyz) - floor(map_nz/2.0)*map_dxyz;
    for (unsigned int z=0; z<map_nz; z++)
    {
        for (unsigned int y=0; y<map_nxy; y++)
        {
            for (unsigned int x=0; x<map_nxy; x++)
            {
                CARTESIAN_CELL cell;
                cell.is_occupied = false;
                cell.sd = 0;
                cell.idx_x = x;
                cell.idx_y = y;
                cell.idx_z = z;
                cell.project2d_idx = x+y*map_nxy;
                cell.log_odds = 0;
                cell.idx = idx_in_order;
                cell.center_pt = Vec3(minxy + (map_dxyz/2) + (x*map_dxyz),
                                      minxy + (map_dxyz/2) + (y*map_dxyz),
                                      minz  + (map_dxyz/2) + (z*map_dxyz));
                cell.relevant_submap_idx = (z/submap_paras.submap_nz)*25+(y/submap_paras.submap_nxy)*5+(x/submap_paras.submap_nxy);
                map->push_back(cell);
                idx_in_order++;
            }
        }
    }
    cout << "local map contain " << map->size() << " cells"<< endl;
}

bool local_map_cartesian::get_the_relevant_submap_for_new_localmap(vector<unsigned int>& relevant_and_occupied_idx,
                                                                   vector<unsigned int>& unrelevant_and_occupied_idx)
{
    relevant_and_occupied_idx.clear();
    unrelevant_and_occupied_idx.clear();
    bool ret=false;
    for (unsigned int i=0; i<125; i++) {
        if(this->sub_maps[i].cells.size()==0) continue;
        Vec3 diff = this->sub_maps[i].submap_info.center_xyz - this->map_center_xyz;
        if(fabs(diff(0))<submap_paras.relevant_submap_search_range_xy &&
                fabs(diff(1))<submap_paras.relevant_submap_search_range_xy &&
                fabs(diff(2))<submap_paras.relevant_submap_search_range_z)
        {
            relevant_and_occupied_idx.push_back(i);
            ret =  true;
        }
        else
        {
            unrelevant_and_occupied_idx.push_back(i);
        }
    }
    return ret;
}

void local_map_cartesian::input_pc_pose(vector<Vec3> PC_hit_a,
                                        vector<Vec3> PC_miss_a,
                                        SE3 T_wa,
                                        map_warehouse* warehouse)
{
    T_wa_latest = T_wa;
    if(first_pose)
    {
        //center_xyz and min_xyz
        double c_x= round(T_wa.translation().x()/map_dxyz)*map_dxyz;
        double c_y= round(T_wa.translation().y()/map_dxyz)*map_dxyz;
        double c_z= round(T_wa.translation().z()/map_dxyz)*map_dxyz;
        map_center_xyz = Vec3(c_x,c_y,c_z);
        map_min_x = map_center_xyz(0) - (0.5*map_dxyz)-floor(map_nxy/2.0)*map_dxyz;
        map_min_y = map_center_xyz(1) - (0.5*map_dxyz)-floor(map_nxy/2.0)*map_dxyz;
        map_min_z = map_center_xyz(2) - (0.5*map_dxyz)-floor(map_nz/2.0)*map_dxyz;
        map_min_xyz = Vec3(map_min_x,map_min_y,map_min_z);
        T_wl =  SE3(SO3(0,0,0),map_center_xyz);
        //init the switching check list
        Vec3 diff_min_center = map_min_xyz - map_center_xyz;
        //cout << "map center: " << T_wl.translation().transpose() <<  endl;

        for(size_t i=0; i<27; i++)
        {
            sub_map_switching_check_list[i].submap_info.center_xyz=Vec3(
                        map_min_x + submap_paras.diff_1st_close_submap_center_localmap_min(0) + (i%3)*map_dxyz*submap_paras.submap_nxy,
                        map_min_y + submap_paras.diff_1st_close_submap_center_localmap_min(1) + ((i%9)/3)*map_dxyz*submap_paras.submap_nxy,
                        map_min_z + submap_paras.diff_1st_close_submap_center_localmap_min(2) + (i/9)*map_dxyz*submap_paras.submap_nz
                        );
            sub_map_switching_check_list[i].submap_info.offset_min_xyz = diff_min_center + sub_map_switching_check_list[i].submap_info.center_xyz;
            //cout << "submap idx: " << i << " center: " << sub_map_switching_check_list[i].submap_info.center_xyz.transpose() << endl;
        }
        first_pose = false;
    }else
    {
        //check whether need to swith to another localmap
        Vec3 t_wa=T_wa.translation();
        double min_distance_2_center = (t_wa-map_center_xyz).norm();
        double min_distance_2_other = 999.0;
        unsigned int min_distance_2_other_idx=27;
        for(size_t i=0; i<27; i++)
        {
            if(i==13)continue;
            double dis = (sub_map_switching_check_list[i].submap_info.center_xyz - t_wa).norm();
            if(dis<min_distance_2_other)
            {
                min_distance_2_other = dis;
                min_distance_2_other_idx = static_cast<unsigned int>(i);
            }
        }
        //if so, switch
        if(min_distance_2_center>min_distance_2_other)
        {
            //            cout << "min_distance_2_center=" << min_distance_2_center << " while " << "min_distance_2_other=" << min_distance_2_other << endl;
            //            cout << "switch to " << min_distance_2_other_idx << endl;
            //STEP1: devided the map to submaps
            this->devide_local_map_to_submaps();
            //STEP2: switching
            map_center_xyz = sub_map_switching_check_list[min_distance_2_other_idx].submap_info.center_xyz;
            map_min_xyz    = sub_map_switching_check_list[min_distance_2_other_idx].submap_info.offset_min_xyz;
            map_min_x = map_min_xyz(0);
            map_min_y = map_min_xyz(1);
            map_min_z = map_min_xyz(2);
            T_wl =  SE3(SO3(0,0,0),map_center_xyz);
            //cout << "new map center: " << T_wl.translation().transpose() << endl;
            for(auto& i: *map)
            {
                i.is_occupied=false;
            }
            //STEP3: recover from warehouse and submaps of previous localmap
            //recover from previous localmap
            vector<unsigned int> relevant, unrelavant, relevant_from_warehouse;
            if(this->get_the_relevant_submap_for_new_localmap(relevant,unrelavant))
            {
                //                cout << "these submap are relevant to the new localmap: ";
                //                for(unsigned int idx:relevant){
                //                    cout << idx << " ";
                //                }
                //                cout << endl;
            }
            for(unsigned int idx:relevant)
            {
                for(SUBMAP_CELL cell : sub_maps[idx].cells)
                {
                    Vec3I xyz_idx;
                    if(xyz2xyzIdxwithBoderCheck(cell.pt_w,xyz_idx))
                    {
                        size_t map_idx=mapIdx(xyz_idx);
                        map->at(map_idx).log_odds=cell.log_odds;
                        map->at(mapIdx(xyz_idx)).is_occupied = true;
                    }
                }
            }
            //recover from warehouse
            if(warehouse->searchSubMap(map_center_xyz,
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


            //recover from warehouse localmap
            sort(relevant_from_warehouse.begin(), relevant_from_warehouse.end());
            for(unsigned int i = 0; i<warehouse->warehouse.size(); i++)
            {
                if(relevant_from_warehouse.size()==0) continue;
                if(warehouse->warehouse[i].submap_info.idx == relevant_from_warehouse.front())
                {
                    relevant_from_warehouse.erase(relevant_from_warehouse.begin());
                    for(SUBMAP_CELL cell : warehouse->warehouse[i].cells)
                    {
                        Vec3I xyz_idx;
                        if(xyz2xyzIdxwithBoderCheck(cell.pt_w,xyz_idx))
                        {
                            size_t map_idx=mapIdx(xyz_idx);
                            map->at(map_idx).log_odds=cell.log_odds;
                            map->at(mapIdx(xyz_idx)).is_occupied = true;
                        }
                    }
                }

            }
            for(unsigned int idx:relevant)
            {
                for(SUBMAP_CELL cell : sub_maps[idx].cells)
                {
                    Vec3I xyz_idx;
                    if(xyz2xyzIdxwithBoderCheck(cell.pt_w,xyz_idx))
                    {
                        size_t map_idx=mapIdx(xyz_idx);
                        map->at(map_idx).log_odds=cell.log_odds;
                        map->at(mapIdx(xyz_idx)).is_occupied = true;
                    }
                }
            }

            //STEP4: init the switching check list
            Vec3 diff_min_center = map_min_xyz - map_center_xyz;
            for(size_t i=0; i<27; i++)
            {
                sub_map_switching_check_list[i].submap_info.center_xyz=Vec3(
                            map_min_x + submap_paras.diff_1st_close_submap_center_localmap_min(0) + (i%3)*map_dxyz*submap_paras.submap_nxy,
                            map_min_y + submap_paras.diff_1st_close_submap_center_localmap_min(1) + ((i%9)/3)*map_dxyz*submap_paras.submap_nxy,
                            map_min_z + submap_paras.diff_1st_close_submap_center_localmap_min(2) + (i/9)*map_dxyz*submap_paras.submap_nz
                            );
                sub_map_switching_check_list[i].submap_info.offset_min_xyz = diff_min_center + sub_map_switching_check_list[i].submap_info.center_xyz;
            }

            //STEP5: add submaps to warehouse & delete submaps in warehouse
            for(unsigned int i=0; i<unrelavant.size(); i++)
            {
                warehouse->addSubMap(sub_maps[unrelavant.at(i)]);
            }
            warehouse->deleteSubMap(relevant_from_warehouse);
        }
    }
    //update map.

    //Frame [w]orld, [s]ensor, [b]ody, [a]wareness, [l]ocalmap;
    vector<Vec3> pc_hit_w;
    vector<Vec3> pc_miss_w;
    for(auto p_a:PC_hit_a)
    {
        pc_hit_w.push_back(T_wa*p_a);
    }
    for(auto p_a:PC_miss_a)
    {
        pc_miss_w.push_back(T_wa*p_a);
    }
    //STEP 2: Add measurement
    for(auto p_w:pc_hit_w)
    {
        Vec3I xyz_idx;
        if(xyz2xyzIdxwithBoderCheck(p_w,xyz_idx))
        {
            size_t map_idx=mapIdx(xyz_idx);
            map->at(map_idx).log_odds+=log_odds_hit;
            if(map->at(map_idx).log_odds > log_odds_max)
            {
                map->at(map_idx).log_odds=log_odds_max;
            }
            //set observerable
            if(map->at(map_idx).log_odds > log_odds_occupied_sh){
                map->at(mapIdx(xyz_idx)).is_occupied = true;
            }
        }else
        {
        }
    }
    for(auto p_miss_w:pc_miss_w)
    {
        Vec3I xyz_idx;
        if(xyz2xyzIdxwithBoderCheck(p_miss_w,xyz_idx))
        {
            size_t map_idx=mapIdx(xyz_idx);
            map->at(map_idx).log_odds--;
            if(map->at(map_idx).log_odds < log_odds_min)
            {
                map->at(map_idx).log_odds=log_odds_min;
            }
            //set free
            if(map->at(map_idx).log_odds < log_odds_occupied_sh){
                map->at(mapIdx(xyz_idx)).is_occupied = false;
            }
        }else
        {
        }
    }
    //Update occupied list;
    occupied_cell_idx.clear();
    for(auto cell: *map)
    {
        if(cell.is_occupied)
        {
            occupied_cell_idx.push_back(cell.idx);
            //visualization_cell_list.push_back(cell.vis_pt);
        }
    }
    //cout << "globalmap vis size" << visualization_cell_list.size() << endl;

}
