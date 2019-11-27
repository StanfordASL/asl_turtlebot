#echo "!!!!!!!!!!!!!!!!!!!!\nEdit this file (~/catkin_ws/src/asl_turtlebot/scripts/rostb3.sh) to replace <turtlebot_name> and <network_interface> below, then comment this line out by preceding it with a #\n!!!!!!!!!!!!!!!!!!!!"
export ROS_MASTER_URI=http://<turtlebot_name>.local:11311
export ROS_IP=$(ifconfig wlp5s0 | grep 'inet' | head -1 | awk '{ print $2}')
export ROS_HOSTNAME=$(ifconfig wlp5s0 | grep 'inet' | head -1 | awk '{ print $2}')

echo "updated settings: "
echo $ROS_MASTER_URI
echo $ROS_IP
echo $ROS_HOSTNAME
