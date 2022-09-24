#!/bin/bash

install_cmake() {
  cd $pwd/3rdparty/cmake/
   tar zxvf cmake-3.17.3.tar.gz
  cd cmake-3.17.3
   ./configure --parallel=$cpu_core
   make -j$cpu_core
  if [[ "$ARM_TOOLCHAIN" == "" ]]; then
     make install
  fi
}

install_3rdparty() {
  cd $pwd	
  mkdir -p build/$target_path/$1 > /dev/null 2>&1
  cd build/$target_path/$1
  #depend_path="-DEIGEN_INCLUDE_DIR=$pwd/3rdparty/eigen/ -DEigen3_DIR=$pwd/build/$target_path/share/eigen3/cmake/"
  depend_path="-DEigen3_DIR=$pwd/build/$target_path/share/eigen3/cmake/ -DEIGEN_INCLUDE_DIR=$pwd/3rdparty/eigen/"
  cmake $pwd/3rdparty/$1 -DCMAKE_BUILD_TYPE=$2 $ARM_CONFIG $depend_path -DCMAKE_INSTALL_PREFIX=$pwd/build/$target_path/
  make -j$cpu_core
  make install
}

install_3rdparty_ceres() {
  cd $pwd
  mkdir -p build/$target_path/ceres-solver > /dev/null 2>&1
  cd build/$target_path/ceres-solver
  #depend_path="-DEIGEN_INCLUDE_DIR=$pwd/3rdparty/eigen/ -DEigen3_DIR=$pwd/build/$target_path/share/eigen3/cmake/"
  #depend_path="-DEigen3_DIR=$pwd/build/$target_path/share/eigen3/cmake/ -DEIGEN_INCLUDE_DIR=$pwd/3rdparty/eigen/ -Dglog_DIR=$pwd/build/$target_path/lib/cmake/glog -Dgflags_DIR=$pwd/build/$target_path/lib/cmake/gflags"

  if [[ $target_path == "pc" ]]; then
     depend_path="-DEigen3_DIR=$pwd/build/$target_path/share/eigen3/cmake/ -DEIGEN_INCLUDE_DIR=$pwd/3rdparty/eigen/ -Dglog_DIR=$pwd/build/$target_path/lib/cmake/glog -Dgflags_DIR=$pwd/build/$target_path/lib/cmake/gflags"
     cmake $pwd/3rdparty/ceres-solver -DCMAKE_BUILD_TYPE=Debug   $ARM_CONFIG $depend_path -DCMAKE_INSTALL_PREFIX=$pwd/build/$target_path/
     make -j$cpu_core
     make install
     cmake $pwd/3rdparty/ceres-solver -DCMAKE_BUILD_TYPE=Release $ARM_CONFIG $depend_path -DCMAKE_INSTALL_PREFIX=$pwd/build/$target_path/
     make -j$cpu_core
     make install
  else
    depend_path="-DEigen3_DIR=$pwd/build/$target_path/share/eigen3/cmake/ -DEIGEN_INCLUDE_DIR=$pwd/3rdparty/eigen/"
    cmake $pwd/3rdparty/ceres-solver -DCMAKE_BUILD_TYPE=Debug   $ARM_CONFIG $depend_path -DCMAKE_INSTALL_PREFIX=$pwd/build/$target_path/ -DMINIGLOG=ON -DLAPACK=OFF -DCUSTOM_BLAS=OFF
    #cmake $pwd/3rdparty/ceres-solver -DCMAKE_BUILD_TYPE=Debug   $ARM_CONFIG $depend_path -DCMAKE_INSTALL_PREFIX=$pwd/build/$target_path/
    make -j$cpu_core
    make install
    cmake $pwd/3rdparty/ceres-solver -DCMAKE_BUILD_TYPE=Release $ARM_CONFIG $depend_path -DCMAKE_INSTALL_PREFIX=$pwd/build/$target_path/ -DMINIGLOG=ON -DLAPACK=OFF -DCUSTOM_BLAS=OFF
    #cmake $pwd/3rdparty/ceres-solver -DCMAKE_BUILD_TYPE=Release $ARM_CONFIG $depend_path -DCMAKE_INSTALL_PREFIX=$pwd/build/$target_path/
    make -j$cpu_core
    make install
  fi
}


install_3rdparty_opencv(){
  cd $pwd	
  mkdir -p build/$target_path/opencv > /dev/null 2>&1
  cd build/$target_path/opencv

  #depend_path="-DEIGEN_INCLUDE_DIR=$pwd/3rdparty/eigen/ -DEigen3_DIR=$pwd/build/$target_path/share/eigen3/cmake/"
  depend_path="-DEigen3_DIR=$pwd/build/$target_path/share/eigen3/cmake/ -DEIGEN_INCLUDE_DIR=$pwd/3rdparty/eigen/"
  echo "@@opencv@@"
  echo "$pwd/build/$target_path/"

  if [[ $target_path == "android" ]]; then
  #https://mdeore.medium.com/opencv-on-android-tiny-with-optimization-enabled-932460acfe38
  ${JAVA_HOME} =  $pwd/build/$target_path/
  cmake $pwd/3rdparty/opencv -D CMAKE_BUILD_TYPE=Release \
  -DANDROID_NATIVE_API_LEVEL=30 \
  -DANDROID_TOOLCHAIN_NAME="llvm" \
  -DANDROID_ABI=arm64-v8a \
  -DWITH_CUDA=OFF \
  -DWITH_MATLAB=OFF \
  -DBUILD_ANDROID_EXAMPLES=OFF \
  -DBUILD_DOCS=OFF \
  -DENABLE_NEON = ON\
  -DCPU_BASELINE=NEON \
  -DBUILD_PERF_TESTS=OFF \
  -DBUILD_TESTS=OFF \
  -DCMAKE_ANDROID_STL_TYPE=c++_shared \
  -DBUILD_SHARED_LIBS=OFF $ARM_CONFIG $depend_path -DCMAKE_INSTALL_PREFIX=$pwd/build/$target_path/
  else
  cmake $pwd/3rdparty/opencv -DCMAKE_BUILD_TYPE=Release $ARM_CONFIG $depend_path -DCMAKE_INSTALL_PREFIX=$pwd/build/$target_path/
  fi
  make -j$cpu_core
  make install
}
install_3rdparty_feature_matching_evo(){
  cd $pwd
  mkdir -p build/$target_path/feature_matching_evo > /dev/null 2>&1
  cd build/$target_path/feature_matching_evo
  depend_path="-DOpenCV_DIR=$pwd/build/$target_path/lib/cmake/opencv4/"
  cmake $pwd/3rdparty/feature_matching_evo -DCMAKE_BUILD_TYPE=Release $ARM_CONFIG $depend_path -DCMAKE_INSTALL_PREFIX=$pwd/build/$target_path/
  make -j$cpu_core
  make install  
}

install_3rdparty_toolchains() {
  apt install -y  python-pip python-tk
  pip install evo --upgrade --no-binary evo
  cd $pwd
  mkdir -p build > /dev/null 2>&1
  #cp -rf 3rdparty/toolchains/slam_trajectory_evaluation build/
}

build() {
   git submodule init
   git submodule update --recursive
  if [[ $target_path == "pc" ]]; then
     apt install -y libpython2.7-dev python-numpy libgtk2.0-dev pkg-config libjpeg-dev libglew-dev
  elif [[ $target_path == "arm_clang" ]]; then 
     apt install -y clang-10 lld-10 lld	  
  fi

  build_type="Release"
  if [[ $2 == "Debug" ]]; then
    build_type=$2
  fi
  echo "$1 $build_type"

  if   [[ $1 == "windows"]] && [[$1 == "toolchains" ]]; then
    return
  elif [[ $1 == "toolchains" ]]; then
    install_3rdparty_toolchains
  elif [[ $1 == "cmake" ]]; then
    install_cmake
  elif [[ $1 == "ceres-solver" ]]; then
    install_3rdparty_ceres
  elif [[ $1 == "opencv" ]]; then
    install_3rdparty_opencv
  elif [[ $1 == "librealsense" ]] && [[ $target_path == "pc" ]]; then
    install_3rdparty $1 $build_type
  elif [[ $1 == "all" ]]; then
     for((i=0; i<${#project_name[@]}; i++))
     do
       if [[ ${project_name[$i]} == "windows" ]] || [[ ${project_name[$i]} == "toolchains" ]]; then
         continue
       elif [[ ${project_name[$i]} == "cmake" ]]; then
         continue
       elif [[ ${project_name[$i]} == "librealsense" ]]; then
         continue
       elif [[ ${project_name[$i]} == "feature_matching_evo" ]]; then
         continue
       elif [[ ${project_name[$i]} == "opencv" ]]; then
         continue
       elif [[ ${project_name[$i]} == "ceres-solver" ]]; then
         continue
       elif [[ ${project_name[$i]} == "pangolin" ]] && [[ $target_path == "arm" ]]; then
         continue
       else
         install_3rdparty ${project_name[$i]} $build_type
       fi
     done
    install_3rdparty_opencv
  if [[ $target_path == "pc" ]]; then
    install_3rdparty_feature_matching_evo
  fi
    install_3rdparty_toolchains
    install_3rdparty_ceres
  else
    for((i=0; i<${#project_name[@]}; i++))
    do
      if [[ $1 == ${project_name[$i]} ]]; then
        install_3rdparty $1 $build_type
      fi
    done
  fi
}
build_clean() {

  if   [[ $1 == "windows"]] && [[$1 == "toolchains" ]]; then
    return
  elif [[ $1 == "cmake" ]]; then
    install_cmake
  elif [[ $1 == "all" ]]; then
    echo "remove $pwd/build/$target_path"
    rm -rf $pwd/build/$target_path
  else
    for((i=0; i<${#project_name[@]}; i++))
    do
      if [[ $1 == ${project_name[$i]} ]]; then
	echo "remove $pwd/build/$target_path/${project_name[$i]}" 
        rm -rf $pwd/build/$target_path/${project_name[$i]}
      fi
    done
  fi
}
