echo "!!!!!!!!!!!!!!!!!!!!\nEdit this file (~/catkin_ws/src/asl_turtlebot/scripts/rostb3.sh) to replace <turtlebot_name> and <network_interface> below, then comment this line out by preceding it with a #\n!!!!!!!!!!!!!!!!!!!!"
export ROS_MASTER_URI=http://<turtlebot_name>.local:11311
export ROS_IP=$(ifconfig <network_interface> | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}')
export ROS_HOSTNAME=$(ifconfig <network_interface> | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}')