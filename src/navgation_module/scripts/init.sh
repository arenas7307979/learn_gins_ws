#!/bin/bash

init_3rdparty() {
  cd $pwd/3rdparty
  i=0
  for FILE in `ls`
  do
      if test -d $FILE
      then
        project_name[$i]=$FILE
        i=$((i+1))
      fi
  done
}
init_test() {
  cd $pwd/test
  i=0
  for MODULE_NAME in `ls`
  do
    if test -d $MODULE_NAME
    then
      test_name[$i]=$MODULE_NAME
      i=$((i+1))
    fi
  done
}
init_base_test() {
  cd $pwd/test/base
  i=0
  for MODULE_NAME in `ls`
  do
    if test -d $MODULE_NAME
    then
      test_base_name[$i]=$MODULE_NAME
      i=$((i+1))
      test_base_name_list="$MODULE_NAME,$test_base_name_list"
    fi
  done
}
init_devices_test() {
  cd $pwd/test/devices
  i=0
  for MODULE_NAME in `ls`
  do
    if test -d $MODULE_NAME
    then
      test_devices_name[$i]=$MODULE_NAME
      i=$((i+1))
      test_devices_name_list="$MODULE_NAME,$test_devices_name_list"
    fi
  done
}
init_image_feature_test() {
  cd $pwd/test/image_feature
  i=0
  for MODULE_NAME in `ls`
  do
    if test -d $MODULE_NAME
    then
      test_image_feature_name[$i]=$MODULE_NAME
      i=$((i+1))
      test_image_feature_name_list="$MODULE_NAME,$test_image_feature_name_list"
    fi
  done
}
init_vins_test() {
  cd $pwd/test/vins
  i=0
  for MODULE_NAME in `ls`
  do
    if test -d $MODULE_NAME
    then
      test_vins_name[$i]=$MODULE_NAME
      i=$((i+1))
      test_vins_name_list="$MODULE_NAME,$test_vins_name_list"
    fi
  done
}
