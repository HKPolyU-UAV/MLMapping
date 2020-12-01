#include "map_warehouse.h"


map_warehouse::map_warehouse()
{
    unique_id = 0;
    warehouse.clear();
}



bool map_warehouse::searchSubMap(Vec3 center_xyz, double dis_sh_xy, double dis_sh_z, vector<unsigned int>& idx)
{
    bool ret=false;
    idx.clear();
    for (auto submap:this->warehouse) {
        Vec3 diff = submap.submap_info.center_xyz - center_xyz;
        if(fabs(diff(0))<dis_sh_xy && fabs(diff(1))<dis_sh_xy && fabs(diff(2))<dis_sh_z)
        {
            idx.push_back(submap.submap_info.idx);
//            cout << submap.submap_info.idx << " in warehouse relevant to localmap" << endl;
//            cout << submap.submap_info.center_xyz.transpose() << endl;
//            cout << center_xyz.transpose() << endl;
//            cout << diff.transpose() << endl;
            ret =  true;
        }
    }
    return ret;
}

void map_warehouse::addSubMap(SUBMAP submap)
{
    submap.submap_info.idx = unique_id;
    unique_id++;
    warehouse.push_back(submap);
    //cout << "add submap with center: " << submap.submap_info.center_xyz.transpose() << endl;
    //cout << "submap in warehouse" << warehouse.size() << endl;
}

void map_warehouse::deleteSubMap(vector<unsigned int> idx)
{
    sort(idx.begin(), idx.end());
    vector<int> delete_idx_high_to_low;
    for(int i = warehouse.size()-1; i>=0; i--)
    {
        if(idx.size()>0)
        {
            if(warehouse[i].submap_info.idx == idx.back() )
            {
                idx.pop_back();
                delete_idx_high_to_low.push_back(i);
            }
        }
        else {
            continue;
        }
    }
    for(int i:delete_idx_high_to_low)
    {
        warehouse.erase (warehouse.begin()+i);
    }
    //cout << "submap in warehouse" << warehouse.size() << endl;
}
