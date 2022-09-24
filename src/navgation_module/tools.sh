#!/bin/bash
cpu_core=$(($(nproc)-1))
pwd=$PWD
time_test=""

source scripts/init.sh
init_3rdparty
init_test
#init_base_test
#init_devices_test
#init_image_feature_test
#init_vins_test

target_path="pc"
target() {
  if [[ $1 == "arm_clang" ]]; then
    ARM_CONFIG=" -DCMAKE_TOOLCHAIN_FILE=$pwd/cmake/toolchain-arm64-clang.cmake -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF"
    target_path=$1
  elif [[ $1 == "arm_gcc" ]]; then
    
    ARM_CONFIG=" -DCMAKE_TOOLCHAIN_FILE=$pwd/cmake/toolchain-arm64-gcc.cmake -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-lstdc++"
    target_path=$1
  elif [[ $1 == "android" ]]; then
    ARM_CONFIG=" -DCMAKE_TOOLCHAIN_FILE=$pwd/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_STL=c++_static -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_JAVA=OFF -DBUILD_ANDROID_EXAMPLES=OFF -DBUILD_ANDROID_PROJECTS=OFF -DENABLE_FAST_MATH=1 -DWITH_CUDA=OFF  -DENABLE_PRECOMPILED_HEADERS=OFF -DANDROID_NDK=/root/Android/Sdk/ndk/22.1.7171670/ .."
    target_path=$1
  elif [[ $1 == "pc"  ]]; then
    target_path=$1
  fi
}
evo=false

if [[ $# == 0 ]]; then

  for((i=0; i<${#project_name[@]}; i++))
  do
    project_name_list="$project_name_list, ${project_name[$i]}"
  done

  for((i=0; i<${#test_name[@]}; i++))
  do
    test_name_list="$test_name_list, ${test_name[$i]}"
  done

  echo -e ""
  echo -e "\e[1;32m\t--target [device]\e[0m"
  echo -e ""
  echo -e "\t  device(Default pc): "
  echo -e ""
  echo -e "\t    \e[1;34mpc, arm_gcc, arm_clang, android\e[0m"
  echo -e ""
  echo -e ""
  echo -e "\e[1;32m\t--build [3rdparty_name] [Release|Debug]\e[0m"
  echo -e ""
  echo -e "\t  3rdparty_name(Default Release): "
  echo -e ""
  echo -e "\t    \e[1;34mall$project_name_list\e[0m"
  echo -e ""
  echo -e "\e[1;32m\t--build_clean [3rdparty_name]\e[0m"
  echo -e ""
  echo -e "\t  3rdparty_name: "
  echo -e ""
  echo -e "\t    \e[1;34mall$project_name_list\e[0m"
  echo -e ""  
  echo -e "\e[1;32m\t--run [test_name] [Debug|Release]\e[0m"
  echo -e ""
  echo -e "\t  test_name(Default Debug): "
  echo -e ""
  echo -e "\t    \e[1;34mslam_online, slam_offline$test_name_list\e[0m"
  echo -e ""
  echo -e "\e[1;32m\t--run_clean [test_name]\e[0m"
  echo -e ""
  echo -e "\t  test_name: "
  echo -e ""
  echo -e "\t    \e[1;34mslam_online, slam_offline$test_name_list\e[0m"
  echo -e ""  
  echo -e "\e[1;32m\t--time\e[0m"
  echo -e ""
  echo -e "\t  full compiler time"
  echo -e ""
  echo -e "\e[1;32m\t--evo\e[0m"
  echo -e ""
  echo -e "\t  vins trajectory visualization"
  echo -e ""  
  echo -e "\e[1;32m\t--login\e[0m"
  echo -e ""
  echo -e "\t  login remote board"
  echo -e ""
else

  source $pwd/scripts/build.sh
  source $pwd/scripts/run.sh
  if [[ $# == 1 ]] && [[ $1 == "--evo" ]]; then
    cd $pwd/build/slam_trajectory_evaluation
    ./run.sh
    return
  fi
  while [[ $# > 0 ]];
  do
    if   [[ $1 == "--target"   ]]; then
      target $2
    elif   [[ $1 == "--build"   ]]; then
      build $2 $3
      break
    elif   [[ $1 == "--build_clean"   ]]; then
      build_clean $2
      break    
    elif [[ $1 == "--run"     ]]; then
      run $2 $3
      break
    elif [[ $1 == "--run_clean"     ]]; then
      run_clean $2
      break      
    elif [[ $1 == "--login"  ]]; then
      login $2
      break
    elif [[ $1 == "--time" ]]; then
      time_test="/usr/bin/time -v"
    elif [[ $1 == "--evo"  ]]; then
      evo=true
    fi  
    shift
  done
fi
