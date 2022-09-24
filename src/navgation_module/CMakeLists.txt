cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_PATH "${CMAKE_SOURCE_DIR}/")
add_definitions(-DPROJECT_PATH="${PROJECT_PATH}")
set(ABSOLUTE_PATH "/mnt/nas_datasets/")
add_definitions(-DABSOLUTE_PATH="${ABSOLUTE_PATH}")
set(TARGET_PATH "pc") #NOTE pc or arm

project(fgo_imu_gnss)
add_compile_options(-std=c++17)

option(ENABLE_SYSTRACE "build systrace " OFF)
if(ENABLE_SYSTRACE)
  add_definitions(-DVICORE_ENABLE_TRACE)
endif()



#--------------------------------------------
#NOTE build sucess pc_x86
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++17 -O3 -lboost_system -msse2 -msse3 -pthread -Wenum-compare -march=native") # -Wall
# set(CMAKE_CXX_FLAGS  "${CMAKE_C_FLAGS} -O3  -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -mavx -mfma -march=native")
# set(CMAKE_CXX_FLAGS  "${CMAKE_C_FLAGS} -O3 -march=native")
# set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
#--------------------------------------------
#NOTE build sucess arm64
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -flax-vector-conversions -fomit-frame-pointer -march=native")
# set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -flax-vector-conversions -fomit-frame-pointer -march=native")
#--------------------------------------------
#NOTE build sucess arm64 /  pc_x86 [sucess build for ob_gins]
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS  "${CMAKE_C_FLAGS} -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -flax-vector-conversions -fomit-frame-pointer -march=native")
# set(CMAKE_CXX_FLAGS "-fsanitize=address ${CMAKE_CXX_FLAGS} $ENV{CXXFLAGS} -fno-omit-frame-pointer -O0 -g -ggdb") #-Wall

# [Test for OB_GINS]
#--------------------------------------------
#NOTE build sucess pc_x86 [sucess build for ob_gins]
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS  "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS_RELEASE} -O3")
# --------------------------------------------
#NOTE debug mode pc_x86 [sucess build for ob_gins]
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS "-fsanitize=address ${CMAKE_CXX_FLAGS} $ENV{CXXFLAGS} -O0 -g -ggdb") #-Wall

#NOTE -----find ros thirdparty
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#NOTE -----find slam 3rdParty
set(gflags_DIR "${PROJECT_PATH}/build/${TARGET_PATH}/lib/cmake/gflags/")
find_package(gflags REQUIRED NO_DEFAULT_PATH)
set(glog_DIR "${PROJECT_PATH}/build/${TARGET_PATH}/lib/cmake/glog/")
find_package(glog REQUIRED NO_DEFAULT_PATH)

message("===glog/gflag_DIR===")
message(${GLOG_INCLUDE_DIR})
message(${GFLAGS_INCLUDE_DIRS})

#NOTE ---- Eigen / Sophus, Ceres
set(EIGEN3_INCLUDE_DIR "${PROJECT_PATH}/3rdparty/eigen")  
set(Sophus_INCLUDE_DIRS "${PROJECT_PATH}/3rdparty/sophus/include")

# find_package(Sophus REQUIRED NO_DEFAULT_PATH)

set(JSONCPP_INCLUDE_DIRS "${PROJECT_PATH}/build/${TARGET_PATH}/include")
set(JSONCPP_LIBRARIES "${PROJECT_PATH}/build/${TARGET_PATH}/lib/libjsoncpp.a")

#NOTE ---- navageion_module(only)
set(YAML-CPP_DIR "${PROJECT_PATH}/build/${TARGET_PATH}/lib/cmake/yaml-cpp")
find_package(YAML-CPP REQUIRED PATHS "${PROJECT_PATH}/build/${TARGET_PATH}/lib/cmake/yaml-cpp")
set(YAML_CPP_INCLUDE_DIR "${PROJECT_PATH}/build/${TARGET_PATH}/include")
set(YAML_CPP_LIBRARIES "${PROJECT_PATH}/build/${TARGET_PATH}/lib/libyaml-cpp.a")

#NOTE ---- set GeographicLib
set(GeographicLib_INCLUDE_DIRS "${PROJECT_PATH}/3rdparty/GeographicLib/include")
set(GeographicLib_LIBRARIES "${PROJECT_PATH}/build/${TARGET_PATH}/lib/libGeographic.a")

# abseil
set(absl_DIR "${PROJECT_PATH}/build/${TARGET_PATH}/lib/cmake/absl/")
find_package(absl REQUIRED NO_DEFAULT_PATH)


set(Ceres_DIR "${PROJECT_PATH}/build/${TARGET_PATH}/lib/cmake/Ceres")
find_package(Ceres REQUIRED NO_DEFAULT_PATH)


#NOTE ----- SLAM_THIRD_PARTY_INCLUDE_DIRS
set(SLAM_THIRD_PARTY_INCLUDE_DIRS
${EIGEN3_INCLUDE_DIR}
${GFLAGS_INCLUDE_DIRS}
${GLOG_INCLUDE_DIR}
${Sophus_INCLUDE_DIRS}
${JSONCPP_INCLUDE_DIRS}
${CERES_INCLUDE_DIRS}
${OPENGL_INCLUDE_DIR}
${YAML_CPP_INCLUDE_DIR}
${GeographicLib_INCLUDE_DIRS}
# ${ZLIB_INCLUDE_DIRS}
)

#NOTE ----- SLAM_THIRD_PARTY_LIBRARIES
set(SLAM_THIRD_PARTY_LIBRARIES
${JSONCPP_LIBRARIES}
${GFLAGS_LIBRARIES}
${GLOG_LIBRARIES}
${CERES_LIBRARIES}
${OPENGL_LIBRARIES}
${YAML_CPP_LIBRARIES}
${GeographicLib_LIBRARIES}
# ${ZLIB_LIBRARIES}
)

message("EIGEN3_INCLUDE_DIR")
message(${EIGEN3_INCLUDE_DIR})
message("Sophus_INCLUDE_DIRS")
message(${Sophus_INCLUDE_DIRS})
message("CERES_INCLUDE_DIRS")
message(${CERES_INCLUDE_DIRS})
message("YAML_CPP_INCLUDE_DIR")
message(${YAML_CPP_INCLUDE_DIR})
message("SLAM_THIRD_PARTY_LIBRARIES")
message(${SLAM_THIRD_PARTY_LIBRARIES})

#NOTE ----- slam core header file
include_directories(
core_lib/include
${SLAM_THIRD_PARTY_INCLUDE_DIRS}
)

#NOTE ----- add slam core lib
cs_add_library(${PROJECT_NAME} SHARED
core_lib/src/start_slam_sys/slam_param_reader.cpp

#NOTE ----SensorIntergration
core_lib/src/slam_vio/integration_base.cpp
#COPY FROM OB_GINS
core_lib/src/slam_vio/bull_integration_base.cpp

#NOTE ----Vsolver
core_lib/src/slam_vio/Vsolver/problem.cpp
core_lib/src/slam_vio/Vsolver/loss_function.cc

#NOTE Vsolver-edge
core_lib/src/slam_vio/Vsolver/edge_imu.cc
core_lib/src/slam_vio/Vsolver/edge_bull_imu.cc
core_lib/src/slam_vio/Vsolver/edge.cc

# gnss edge
core_lib/src/slam_vio/Vsolver/edge_gnss.cc
core_lib/src/slam_vio/Vsolver/edge_imu_last_bias.cc


#NOTE Vsolver-vertex
core_lib/src/slam_vio/Vsolver/vertex_pose.cc
core_lib/src/slam_vio/Vsolver/vertex.cc
core_lib/src/slam_vio/Vsolver/vertex_so3.cc

#NOTE ----CereSolverJ
core_lib/src/slam_vio/ceres/imu_factor.cpp
core_lib/src/slam_vio/ceres/ceres_bull_imu_factor.cpp
core_lib/src/slam_vio/ceres/gnss_ceres_factor.cpp
core_lib/src/slam_vio/ceres/local_parameterization_notz_se3.cpp
core_lib/src/slam_vio/ceres/local_parameterization_se3.cpp
core_lib/src/slam_vio/ceres/local_parameterization_so3.cpp
# core_lib/src/slam_vio/ceres/marginalization_factor.cpp

#NOTE ----OB_GINS
core_lib/src/ob_gins/preintegration/preintegration_base.cc
core_lib/src/ob_gins/preintegration/preintegration_earth_odo.cc
core_lib/src/ob_gins/preintegration/preintegration_earth.cc
core_lib/src/ob_gins/preintegration/preintegration_normal.cc
core_lib/src/ob_gins/preintegration/preintegration_odo.cc
)
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES} ${YAML_CPP_LIBRARIES} ${OpenCV_LIBS})


#####[TODO] NOTE ------ 20220513_Stage1_FGO_IMU_GPS -------
# cs_add_executable(fgo_gps_imu_node
# ros_test/ros_sensor_input/DataLoader_fgo_gps_imu.cpp
# core_lib/src/fgo_gps_imu/fgo_gps_imu_solver.cpp
# )
# target_link_libraries(fgo_gps_imu_node ${PROJECT_NAME} ${SLAM_THIRD_PARTY_LIBRARIES})



#####[TODO] NOTE ------ OBGINS_EXAMPLE -------
cs_add_executable(ob_gins_node
core_lib/src/ob_gins/fileio/fileloader.cc
core_lib/src/ob_gins/fileio/filesaver.cc
ros_test/ros_sensor_input/ob_gins.cc
core_lib/src/fgo_gps_imu/fgo_gps_imu_solver.cpp
)
target_link_libraries(ob_gins_node ${PROJECT_NAME} ${SLAM_THIRD_PARTY_LIBRARIES} absl::strings absl::str_format absl::time)



#####[TODO] NOTE ------OfflineDataLoaderForIphone -------
cs_add_executable(iphone_data_loader_node
ros_test/ros_sensor_input/DataLoader_fgo_gps_imu.cpp
core_lib/src/fgo_gps_imu/fgo_gps_imu_solver.cpp
)
target_link_libraries(iphone_data_loader_node ${PROJECT_NAME} ${SLAM_THIRD_PARTY_LIBRARIES})




#NOTE #NOTE  Install project namespaced headers
install(DIRECTORY include/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.hpp"
PATTERN ".svn" EXCLUDE)

install(DIRECTORY include/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.hpp"
PATTERN ".svn" EXCLUDE)

cs_install()
cs_export()
