# MLMapping(Multilayer Mapping Kit)-Embedded version

### Use it with a planner
You can acquire the information of one global position in the world from the map data structure directly with this **Embedded version**. Just build this package first in your workspace, include the header file ````#include <mlmap.h>```` in your project, and use the interface functions. The initialization only need the ros nodehandle, and it receive the ros messages automatically to build the map. Check nodelet_map.cpp for the detail guidance.

**Interface functions** 

Set the status of one piece of the entire map into free (input the min and max corner coordinate of the region):
````

mlmap::setFree_map_in_bound(Vec3 box_min, Vec3 box_max);
````
Get the status of the queried position (return -1 for unknown, 1 for free, 0 for occupied):
````
int mlmap::getOccupancy(Vec3 pos_w);
````

Get the odds of 'occupied' of the queried position:
````
float mlmap::getOdd(Vec3 pos_w);
````

Get the gradient of the odds of the queried position (the gradient is a vector heading to the neighboring cell of the lowest odds of occupancy):
````
Vec3 mlmap::getOddGrad(Vec3 pos_w);
````

### Introduction
**MLMapping** is a multilayer mapping framework designed for autonomous UAV navigation applications. In this framework, we divided the map into three layers: awareness, local, and global. The awareness map is constructed on the cylindrical coordinate, which enables fast raycasting. The local map is a probability-based volumetric map. The global map adopts dynamic memory management, allocating memory for the active mapping area, and recycling the memory from the inactive mapping area. The framework supports different kinds of map outputs for the global or local path planners.

### Videos:
<a href="https://www.youtube.com/embed/kBLQzIB_kWo" target="_blank"><img src="http://img.youtube.com/vi/kBLQzIB_kWo/0.jpg" 
alt="cla" width="480" height="300" border="1" /></a>

| Fast Raycasting    | Large Scale Mapping   | Autonomous UAV Navigation  |
| ---------------------- | ---------------------- |---------------------- |
| <img src="others/exp1.gif" width="250">  | <img src="others/exp2.gif" width="250">  | <img src="others/exp3.gif" width="250">  |

### Publications
[Chen, S., Chen, H., Chang, C. W., & Wen, C. Y. (2021). Multilayer Mapping Kit for Autonomous UAV Navigation. IEEE Access.](https://ieeexplore.ieee.org/abstract/document/9336584)
### Compile
Clone this repository to catkin src folder say: ~/catkin_ws/src
````
cd ~/catkin_ws/src
git clone https://github.com/HKPolyU-UAV/MLMapping.git
````
Install 3rd Part library
````
cd catkin_ws/src/MLMapping/3rdPartLib/
./install3rdPartLib.sh
````
Compile
````
cd ~/catkin_ws/
catkin_make
````

### Verify Using Provided Dataset
Download the [Large Scale Mapping Dataset](https://connectpolyu-my.sharepoint.com/:u:/g/personal/17903070r_connect_polyu_hk/EYGhc0ijYl9Muq33mUSCgxABZgNBJrTQPp34SY65gWoXRA?e=8lkjxb) into the bag folder <br />
decompress the rosbag
````
rosbag decompress corridor.bag
````
run 
````
roslaunch mlmapping rviz.launch
roslaunch mlmapping mlmapping_bag_l515_t265.launch
````

### Maintainer
[Shengyang Chen](https://www.polyu.edu.hk/researchgrp/cywen/index.php/en/people/researchstudent.html)(Dept.ME,PolyU): shengyang.chen@connect.polyu.hk <br />

