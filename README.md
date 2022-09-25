# learn_gins_ws
## To be solved : vsolver and template ba_solver are unstable


##Select linear Solver (learn_gins_ws/src/navgation_module/core_lib/src/fgo_gps_imu/fgo_gps_imu_solver.cpp)
```
// 0 : ceres solver, 1 : vsolver, 2: template ba_solver
#define CERES_SOLVE_BULL_IMU 0

```

## build 3rdParty
```
step1. cd learn_gins_ws/src/navgation_module
step2. ./tools.sh --build all
step3. cd ../../
step3. catkin build
step4. source devel/setup.bash
```

## run iphone_datatest (ros display)
```
[terminal1] step1. roscore 

[terminal2] step2. rosrun fgo_imu_gnss iphone_data_loader_node src/navgation_module/config/imu_gps_fuse_datasets/mobile_one_for_all_withgps.yaml src/navgation_module/config/2022-09-17T16-19-16/SyncAccGyro.txt src/navgation_module/config/2022-09-17T16-19-16/GPS.txt 

[terminal3] step3. Start rviz and select display_file [src/navgation_module/config/imu_gps_fuse_datasets/gnss_imu_ros_display.rviz]
```


## run gins_dataset (ros display)
```
[terminal1] step1. roscore 

[terminal2] step2. rosrun fgo_imu_gnss ob_gins_node src/navgation_module/config/ob_dataset/ob_gins.yaml

[terminal3] step3. Start rviz and select display_file [src/navgation_module/config/imu_gps_fuse_datasets/gnss_imu_ros_display.rviz]
```


## Acknowledgments
```
OB_GINS : https://github.com/i2Nav-WHU/OB_GINS
Graph-Optimizor : https://github.com/Horizon1026/Graph-Optimizor
thanks for providing.
```


