algorithm=$1
dataset_name=$2
evo_traj tum --ref=gt/${dataset_name}.tum $algorithm/${dataset_name}.tum -va -p --plot_mode xyz
