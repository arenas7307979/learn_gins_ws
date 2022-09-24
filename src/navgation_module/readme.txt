
-------------------Using ROS datasets-----------------------------

1. build 3rd party under path of /vicore_ws/src/navgation_module
   step1: ./tools.sh --build_clean all 
   step2: ./tools.sh --build all


2. build ros project under path of /vicore_ws/
   step1: catkin init 
   step2: catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
   step3: catkin config --merge-devel
   step4: catkin build
   step5: source devel/setup.bash


3. change yaml path (slam_parameters) 
   ros_test/ros_sensor_input/RGBDmain.cpp --> line 18
   ==>slam_param_yaml_path = "/datasets/vicore_ws/src/navgation_module/config/d435i.yaml";

4. roslaunch slam_corevis RGBDxxx.launch

5. rosbag play xxxRGBD.bag


6. ROSInterface ros_test/ros_sensor_input/RGBDmain.cpp ---> slam_ros_display.cpp[display] ---> system(pub sensor info to slam) ---> SLAM 

-------------- Disable ROS, using d435i -----------------------------
0. under path of slam_core ./tools.s --run slam_system_v1_d435i

-------------- SLAM configure -----------------------------
1. backend wheel encoder disable : "WHEEL_ENCODER_ON 0" in vsolver_rgbd_encoder_TBA.h
2. Display Pangoline or ROS enable : "//#define USE_DISPLAY--> #define USE_DISPLAY" in system.h
3. SYSTRACE_FLAG_ENABLE ON/OFF::
   under path of cmake/slam_cmake: option(ENABLE_SYSTRACE "build systrace " OFF) --> ON
4. add PCGSolver to Vsolver/problem.cpp --> #define USE_PCG_SOLVER 1 OR 0[disable]
