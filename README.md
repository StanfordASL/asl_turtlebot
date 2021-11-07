# asl_turtlebot

This contains a _starting point_ for your final project. Below are _brief_
descriptions of the code. You are strongly encouraged to take a closer look into
the code for more details of how and what the code does.

**File Descriptions:**

**Gazebo Simulation Files:**
----------------------

`world/signs.world`: Defines 3D model stop sign testing environment.

`world/project_city.world`: Defines 3D model of rough, tentative
representation of the final project environment.

`world/maze.world`: Defines 3D model of the maze world. For HW4.

`world/arena.world`: Defines 3D model of the arena world. For HW4.

**Turtlebot Files:**
----------------------
**Launch Files:**

`launch/asl_turtlebot_core.launch`: Launches the core elements of the
turtlebot stack (turtlebot drivers, camera, lidar, gmapping, static tf
transforms). This should run onboard the jetson.

`launch/root.launch`: The main configurable launch file on top of
which the remaining launch files are built. Can be launched with a simulator or
on top of the running hardware turtlebot.

`launch/project/project.launch`: Launches gazebo with a (rough, tentative)
model of the final project environment, as well as the core SLAM and detector
nodes. You'll need to run your navigator and other project nodes separately.

`launch/section4_demo.launch`: used for section 4, testing pose
navigation, virtual

`launch/homeworks/hw3.launch`: launches the world for HW3.

`launch/homeworks/hw4_arena.launch`: launches the world for ekf_slam.

`launch/homeworks/hw4_maze.launch`: launches the world for localization.py.

**Scripts/Nodes:**

`scripts/camera_relay.py`: Due to networking limitations, your remote
machine cannot access messages directly from the raspberry pi. This node
forwards images from the raspberry pi via the jetson to the remote machine.

`scripts/goal_commander.py`: Translates Rviz nav goal clicks
(/move_simple_base/goal) to the /cmd_nav topic.

`scripts/detector.py`: Gazebo stop sign detector from HW2. Publishes to
/detector/* where * is the detected object label.

`scripts/detector_mobilenet.py`: Runs tensorflow mobilenet model for image
classification. Publishes to /detector/* where * is the detected object label.
**DISCLAIMER:** The distance estimation is not always very accurate and is
noisy. It subscribes to the /scan which takes the closest point (in xy-distance)
from any laserscan ring below the horizontal ring, ignoring all points a
threshold z_min below the velodyne as ground points. For the current
configuration of the Turtlebot, we have set z_min = 16cm. You can combine the
camera and/or point cloud to improve the estimate of the distance.

`scripts/detector_viz.py`: Visualizes camera feed, bounding boxes and
confidence for detected objects.

`scripts/utils/grids.py`: Used for motion planning. Performs collision checking on
occupancy grids. grids.py functions/classes are used by scripts/navigator.py.

`scripts/navigator.py`: Node that manages point to point robot navigation, uses
your A\* implementation (HW2) in an MPC framework along with cubic spline
interpolation and the differential flatness controller (from HW1), switching to
the pose controller from HW1 when close to the goal.

`scripts/utils/utils.py`: Utility functions. Currently contains a wrapToPi
function, but feel free to add to this.

**Rviz Configurations:**

**Cfg Files:**

`navigator.cfg`: python script to generate dynamic parameter reconfigure for
navigator

`cfg/gripper_control.yaml`: multiple variable configuration for the gripper

***Files From HW***

scripts/controllers/ should contain `P1_pose_stabilization.py` and
`P2_trajectory_tracking.py` from HW1 

scripts/planners/ should contain `P1_astar.py` from HW2

scripts/ should contain the folder `HW4` from HW4`

**Message Definitions:**

`msg/DetectedObject.msg`: Custom message type that describes detected objects.
Contains the following fields:

uint32 id - Label identifying number

string name - Name of identified object

float64 confidence - Classification probability

float64 distance - Distance to object (**DISCLAIMER:** current implementation
relies on /scan topic for distance (see detector.py and detector_mobilenet.py).
The distance estimation for works in gazebo for stop signs (HW2) but not tested
on hardware)

float64 thetaleft - Left bounding ray of object.

float64 thetaright - Right bounding ray of object.

float64[] corners - Corners of bounding box around detected object with respect
to the tf camera frame.

msg/DetectedObjectList.msg`: Custom message type consisting of a
list/array of DetectedObject objects and their names. Contains the following
fields:

string[] objects - Array of strings corresponding to object names.

DetectedObject[] ob_msgs - Array of DetectedObject objects.


**Tensorflow Models:**

The `.pb` files in the `tfmodels` folder are "frozen" neural network models, and
contain both the structure and the weights of pretrained networks.
`ssd_mobilenet_v1_coco.pb` is a pretrained MobileNet v1 model, while
`stop_sign_gazebo.pb` is a model fine-tuend to detect stop signs in Gazebo. We
recommend using `ssd_resnet_50_fpn.pb`, which is a larger, more accurate and
robust model, but does not fit on a GitHub repo and can be downloaded
[here](https://stanford.app.box.com/s/vszjfhwkjb203qbwhzoirn3uzt5r16lv).

The `coco_labels.txt` file just contains the mapping from the class number
output by the model to human-interpretable labels.

There are many other pretrained models you could use, see the [Tensorflow
detection model
zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md)
for more.


**Other:**

`env_pi.sh`: Script to remote launch nodes on the raspberry pi from the jetson.
This overcomes the need to ssh into the raspberry pi separately from the jetson
to launch the camera node. This goes in ~/catkin_ws/devel/ on the raspberry pi.

`roslocal.sh`, `rostb3.sh`: Scripts to set your ROS IP settings.

`CMakeLists.txt`: CMake file for the package
