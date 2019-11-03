HOSTNAME=$(hostname)
if [[ ${HOSTNAME} == "genbu.stanford.edu" ]]; then
  export ROS_MASTER_URI=http://localhost:11114
else
  export ROS_MASTER_URI=http://localhost:11311
fi
export ROS_IP=127.0.0.1
export ROS_HOSTNAME=127.0.0.1
