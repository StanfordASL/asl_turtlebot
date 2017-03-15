# asl_turtlebot

## AA274 Project Notes

### Simulation
* To start up the gazebo simulation environment and rviz map visualization, simply run

  ```bash
  roslaunch asl_turtlebot turtlebot_project_sim.launch
  ```

* To run gazebo "headless" without actually displaying the simulation window (improves the simulation speed on old computers and enhances the realism of our "Mars" mission!), run

  ```bash
  roslaunch asl_turtlebot turtlebot_project_sim.launch gui:=false headless:=true
  ```

### Hardware

* Networking
  * To connect to the turtlebots in the ASL, you must be on the `NETGEAR19` wifi network (preferably `NETGEAR19-5G`, to avoid network outages when the microwave is running). The password is `turtlebot`.
  * If you are running Ubuntu within a virtual machine (i.e., the AA274 VM), you must change your VM network settings (found in the VM-specific settings, not the general VMware settings) to "Bridged Networking."
  * At the bottom of your `~/.bashrc` you must add the lines

    ```bash
    export ROS_IP=$(ifconfig ens33 | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}')
    export ROS_MASTER_URI=http://192.168.1.222:11311
    # export ROS_MASTER_URI=http://$ROS_IP:11311
    ```

    * Replace `ens33` with whatever network interface is populated with interesting looking numbers when you run the command `ifconfig` in a terminal.
    * Replace `192.168.1.222` with `192.168.1.223` or `192.168.1.224` depending on the which turtlebot you're connecting to (see the labels on the lid of the laptops).
    * If you want to go back to testing in Gazebo and running the `roscore` locally in your VM, uncomment the last line.
  * Be sure to `source ~/.bashrc` after any changes!
* Running gmapping onboard the turtlebot
  * In terminal windows on the turtlebot laptop (or more conveniently for starting/stopping, through ssh terminals at `ssh turtlebot@192.168.1.22x`, password `turtlebot`), run:

    ```bash
    roslaunch asl_turtlebot turtlebot_bringup.launch
    ```
  
    and
  
    ```bash
    roslaunch asl_turtlebot turtlebot_onboard.launch
    ```
  * Now on your personal machine, run `rviz -d ~/catkin_ws/src/asl_turtlebot/rviz/mission.rviz` to see the beginnings of a map!
