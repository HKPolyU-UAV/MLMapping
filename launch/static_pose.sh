rostopic pub /imu_pose geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 0, y: 0, z: 0}, orientation: {w: 1.0}}}' -r 30 -s
