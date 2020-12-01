#ifndef MAP_WAREHOUSE_H_
#define MAP_WAREHOUSE_H_

#include <utils/include/all_utils.h>
#include <data_type.h>


class map_warehouse
{
    unsigned int unique_id;
public:
    vector<SUBMAP> warehouse;

    map_warehouse();
    void addSubMap(SUBMAP submap);
    void deleteSubMap(vector<unsigned int> idx);
    bool searchSubMap(Vec3 center_xyz, double dis_sh_xy, double dis_sh_z, vector<unsigned int>& idx);
};

#endif // MAP_WAREHOUSE_H
