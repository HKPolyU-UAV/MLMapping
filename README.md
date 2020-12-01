# MLMapping(Multilayer Mapping Kit)
### Introduction


### Demo
TBD

### Publications
TBD

### Usage
Clone this repository to catkin src folder say: ~/catkin_ws/src
````
cd ~/catkin_ws/src
git clone https://github.com/HKPolyU-UAV/MLMapping.git
````
Install 3rd Part library
````
cd catkin_ws/src/glmapping/3rdPartLib/
./install3rdPartLib.sh
````
Compile
````
cd ~/catkin_ws/
catkin_make
````
Download the [Dataset](https://drive.google.com/file/d/1AF0zBQUizYWccYE9hravpHns8beP1a0Z/view?usp=sharing) into the bag folder <br />
decompress the rosbag
````
rosbag decompress glmapping_test.bag
````
run 
````
roslaunch glmapping bag.launch
````
### Maintainer
[Shengyang Chen](https://www.polyu.edu.hk/researchgrp/cywen/index.php/en/people/researchstudent.html)(Dept.ME,PolyU): shengyang.chen@connect.polyu.hk <br />

