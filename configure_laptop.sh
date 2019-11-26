#!/usr/bin/env bash
sudo apt-get -qq update
echo "initial update...done"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get -qq update -y
echo "Installing ROS Melodic..."
sudo apt install ros-melodic-desktop-full -y
echo "Done."
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get -qq install ros-melodic-catkin python-catkin-tools -y
sudo apt-get -qq install python-rosinstall python-rosinstall-generator python-wstool build-essential -y
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make -quiet
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "Check ROS_PACKAGE_PATH below (should be /home/aa274/catkin_ws/src:/opt/ros/melodic/share)"
echo $ROS_PACKAGE_PATH
echo "Installing Atom..."
wget -qO - https://packagecloud.io/AtomEditor/atom/gpgkey | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64] https://packagecloud.io/AtomEditor/atom/any/ any main" > /etc/apt/sources.list.d/atom.list'
sudo apt-get -qq update
sudo apt-get -qq install atom -y
echo "Done."
echo "Installing essential Python packages..."
sudo apt-get -qq install python-numpy python-scipy python-matplotlib ipython python-pandas python-sympy python-nose -y
echo "Done."
echo "Updating Gazebo...."
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get -qq install gazebo9 -y
echo "Done."
echo "git cloning repos"
rosrun rqt_reconfigure rqt_reconfigure
sudo apt-get -qq install ros-melodic-slam-gmapping -y
sudo apt-get -qq install ros-melodic-gmapping -y
cd ~/catkin_ws/src/
git clone --quiet https://github.com/StanfordASL/asl_turtlebot.git
echo "alias rostb3='source ~/catkin_ws/src/asl_turtlebot/rostb3_melodic.sh'" >> ~/.bashrc
echo "alias roslocal='source ~/catkin_ws/src/asl_turtlebot/roslocal.sh'" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
rostb3
cd ~/catkin_ws/src/
git clone --quiet https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
cd ~/catkin_ws/src/
git clone --quiet https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/catkin_ws && catkin_make
echo "All done. Don't forget to update rostb3_melodic."
