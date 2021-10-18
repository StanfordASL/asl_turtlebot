# user variables section #######################################################
# TURTLEBOT_NAME=<insert your turtlebot name here, e.g. "asteroid">
# e.g. TURTLEBOT_NAME=arthur
TURTLEBOT_NAME="" # TODO insert your value in quotes
################################################################################

# user variables validation section ############################################
if [ -z "$TURTLEBOT_NAME" ]; then
  echo "######################## Error! ########################"
  echo ""
  echo "Edit this file (~/catkin_ws/src/asl_turtlebot/rostb3.sh)" \
    "to populate the TURTLEBOT_NAME variable"
  echo ""
  echo "######################## Error! ########################"
  return 1
fi
################################################################################

# rest of the script needs not changes #########################################

# extracts the first (usually there's only one) linux wireless interface's name
INTERFACE_NAME=$(ifconfig | grep -Eo '^wl[0-9a-Z]+' | head -n1)

# sets the ROS_MASTER_IP to be the robot
export ROS_MASTER_URI="http://$TURTLEBOT_NAME.local:11311"

# sets the own hostname and ip to match the local wireless LAN ip
export ROS_IP=$(ifconfig $INTERFACE_NAME | grep -Eo '192\.168\.[0-9]+\.[0-9]+' \
  | head -n1)
export ROS_HOSTNAME=$ROS_IP

echo "Success!"
echo "Set the following values:"
echo "    ROS_MASTER_URI=$ROS_MASTER_URI"
echo "    ROS_HOSTNAME=$ROS_HOSTNAME"
echo "    ROS_IP=$ROS_IP"
echo "Optionally check that ROS_HOSTNAME and ROS_IP match this machine's ip"
################################################################################
