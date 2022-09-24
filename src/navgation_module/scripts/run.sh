#!/bin/bash

run_test() {
  cd $pwd
  mkdir -p build/$target_path/test/$1/ 2>&1
  cd build/$target_path/test/$1/
  cmake $pwd/test/$1 -DCMAKE_BUILD_TYPE=$2 $ARM_CONFIG -DPROJECT_PATH=$pwd -DTARGET_PATH=$target_path
  if [[ $target_path == "pc" ]]; then
    $time_test  make -j$cpu_core &&
    echo -e "" &&
    echo -e "\e[1;35mRun $1_test\e[0m" &&
    cd $pwd &&
    build/$target_path/test/$1/$1"_test"
    if [[ $evo == true ]]; then
      cd build/slam_trajectory_evaluation
      ./run.sh
    fi
  elif [[ $target_path == "arm_clang" ]] || [[ $target_path == "arm_gcc" ]]; then
     make -j$cpu_core &&
     sshpass -p su scp -rC $pwd/build/$target_path/test/$1/* root@$raspi_ip:/work/slam/build/
  fi
}

run_system() {
   mkdir -p $pwd/build/$target_path/$1 2>&1
  cd $pwd/build/$target_path/$1
   cmake $pwd/src/$1 .. -DCMAKE_BUILD_TYPE=$2 $ARM_CONFIG -DPROJECT_PATH=$pwd -DTARGET_PATH=$target_path
  if [[ $target_path == "arm_clang" ]] || [[ $target_path == "arm_gcc" ]]; then
     $time_test make -j$cpu_core
    cd $pwd
     sshpass -p su scp -rC $pwd/build/$target_path/$1/* root@$raspi_ip:/work/slam/build/
     mkdir -p $pwd/build/pc/slam_udp_receiver 2>&1
    cd $pwd/build/pc/slam_udp_receiver
     cmake $pwd/src/slam_udp_receiver .. -DCMAKE_BUILD_TYPE=$2 -DPROJECT_PATH=$pwd -DTARGET_PATH="pc"
     $time_test make -j$cpu_core && echo -e "" &&
    echo -e "\e[1;35mRun $1 \e[0m" &&
     $pwd/build/pc/slam_udp_receiver/slam_udp_receiver    
  elif [[ $target_path == "pc"  ]]; then
     $time_test make -j$cpu_core &&
    echo -e "\e[1;35mRun $1 \e[0m" &&
     $pwd/build/pc/$1/$1
  fi
}

run() {
  mkdir /mnt/homes > /dev/null 2>&1
  build_type="Debug"
  if [[ $2 == "Release" ]]; then
    build_type=$2
  fi

  if [[ $1 == "slam_online" ]] || [[ $1 == "slam_offline" ]]; then
    run_system "$1_system" $build_type
  else
    for((i=0; i<${#test_name[@]}; i++))
    do
      if [[ $1 == ${test_name[$i]} ]]; then
        run_test $1 $build_type
      fi
    done
  fi
}

run_clean() {
  if [[ $1 == "slam_online" ]] || [[ $1 == "slam_offline" ]]; then
    echo "remove $pwd/build/$target_path/$1"
    rm -rf $pwd/build/$target_path/$1
  else
    echo "remove $pwd/build/$target_path/test/$1"
    rm -rf $pwd/build/$target_path/test/$1 
  fi
}
login() {
  sshpass -p su ssh -X root@$raspi_ip
}

