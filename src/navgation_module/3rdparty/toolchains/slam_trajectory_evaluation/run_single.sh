#!/bin/bash

if [[ $# == 0 ]]; then
  echo "No input file"
  exit 0
fi
BASEDIR=$(dirname "$0") 
cd $BASEDIR
echo "$BASEDIR"
mkdir -p results/$1
rm results/$1/*

for dir in */
do
    if [ $dir != 'gt/' ] && [ $dir != 'results/' ]
    then
        cd $dir
	evo_ape tum ../gt/$1.tum $1.tum -va --save_results ../results/$1/${dir%/*}.zip
	cd ..
    fi
done

cd results
evo_res $1/*.zip -p --use_filenames  --save_table $1/table.csv
