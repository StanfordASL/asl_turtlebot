# asl_turtlebot

This is the PROJECT branch of the asl_turtlebot repo. This contains a _starting point_ for your final project. There are some files overlapping with those existing in the MASTER branch, but this branch is meant to be an organized collection of files for your project and works in isolation. Below are _brief_ descriptions of the code. You are strongly encouraged to take a closer look into the code for more details of how and what the code does.

**File Descriptions:**

**Gazebo Simulation Files:**
----------------------
launch/project_sim_pose.launch: Launches gazebo with a (rough, tentative) model of the final project environment. Motion planning not implemented. It goes to /cmd_pose using pose_controller.py (does not avoid obstacles).

launch/project_sim_nav.launch: Launches gazebo with a (rough, tentative) model of the final project environment. Uses navigator.py but you will need to add your implementation of astar.py for this to work. It goes to /cmd_nav by computing a path.

robots/asl_turtlebot.urdf.xacro, turtlebot3_burger.gazebo.xacro, turtlebot3_burger.urdf.xacro: Turtlebot 3D model definitions for gazebo. 

world/project_city.world: Defines 3D model of rough, tentative representation of the final project environment.


**Turtlebot Files:**
----------------------
**Launch Files:**

launch/turtlebot3_bringup_jetson_pi.launch: Launches the core elements of the turtlebot stack (turtlebot drivers, camera, lidar, gmapping, static tf transforms). This should run onboard the jetson.

launch/velodyne_filter.launch: An example of how to filter the point cloud based on spatial coordinates and reflected beam intensity. This is used in conjunction with puddle_viz.py. You can run "rosrun rqt_reconfigure rqt_reconfigure" to change the threshold parameters online.


**Scripts/Nodes:**

scripts/camera_relay.py: Due to networking limitations, your remote machine cannot access messages directly from the raspberry pi. This node forwards images from the raspberry pi via the jetson to the remote machine.

scripts/cmd_pose_commander.py: Translates Rviz nav goal clicks (/move_simple_base/goal) to the /cmd_pose topic.

scripts/cmd_nav_commander.py: Translates Rviz nav goal clicks (/move_simple_base/goal) to the /cmd_nav topic.

scripts/detector.py: Gazebo stop sign detector from HW2. Publishes to /detector/* where * is the detected object label.

scripts/detector_mobilenet.py: Runs tensorflow mobilenet model for image classification. Publishes to /detector/* where * is the detected object label. **DISCLAIMER:** The distance estimation is not always very accurate and is noisy. It subscribes to the /scan which takes the closest point (in xy-distance) from any laserscan ring below the horizontal ring, ignoring all points a threshold z_min below the velodyne as ground points. For the current configuration of the Turtlebot, we have set z_min = 16cm. You can combine the camera and/or point cloud to improve the estimate of the distance.

scripts/detector_resnet.py: Same as scripts/detector_mobilenet.py but uses the ResNet model instead. It is a more sophisticated model than mobilenet. We found this is better at detecting food objects, but it is up to you to choose which one you want.

scripts/detector_viz.py: Visualizes camera feed, bounding boxes and confidence for detected objects.

scripts/grids.py: Used for motion planning. Performs collision checking on occupancy grids. grids.py functions/classes are used by scripts/navigator.py.

scripts/keyboard_teleop.py: Alternative teleoperation to standard turtlebot3_teleop.launch. 

scripts/navigator.py: Path planner that will use A* implementation (to be completed as part of HW4) in an MPC framework along with cubic spline interpolation and the differential flatness controller (from HW1).

scripts/pose_controller.py: Pose controller from HW1.

scripts/puddle_viz.py: Subscribes to the filtered velodyne points produced by velodyne_filter.launch (/velodyne_puddle_filter). Identifies approximate location of a highly reflective region on the ground (i.e. puddles) and places a marker and tf frame over it. **DISCLAIMER:**  This is very rough starter code meant to serve as an example for point cloud processing. This code is not robust and is not integrated with any of the existing stack. For example, this node cannot deal with multiple puddles simultaneously and estimates location naively. This is by no means the only or expected approach to identifying puddles and your group is expected to entirely repurpose or rewrite this code to suit your team's objectives.

scripts/utils.py: Utility functions. Currently contains a wrapToPi function, but feel free to add to this.


**Message Definitions:**

msg/DetectedObject.msg: Custom message type that describes detected objects. Contains the following fields:

uint32 id - Label identifying number

string name - Name of identified object

float64 confidence - Classification probability

float64 distance - Distance to object (**DISCLAIMER:** current implementation relies on /scan topic for distance (see detector.py and detector_mobilenet.py). The distance estimation for works in gazebo for stop signs (HW2) but not tested on hardware)

float64 thetaleft - Left bounding ray of object.

float64 thetaright - Right bounding ray of object.

float64[] corners - Corners of bounding box around detected object with respect to the tf camera frame.

msg/DetectedObjectList.msg: Custom message type consisting of a list/array of DetectedObject objects and their names. Contains the following fields:

string[] objects - Array of strings corresponding to object names.

DetectedObject[] ob_msgs - Array of DetectedObject objects.


**Tensorflow Models:**

The `.pb` files in the `tfmodels` folder are "frozen" neural network models, and contain both the structure and the weights of pretrained networks. `ssd_mobilenet_v1_coco.pb` is a pretrained MobileNet v1 model, while `stop_sign_gazebo.pb` is a model fine-tuend to detect stop signs in Gazebo. We recommend using `ssd_resnet_50_fpn.pb`, which is a larger, more accurate and robust model, but does not fit on a GitHub repo and can be downloaded [here](https://stanford.app.box.com/s/vszjfhwkjb203qbwhzoirn3uzt5r16lv). 

The `coco_labels.txt` file just contains the mapping from the class number output by the model to human-interpretable labels.

There are many other pretrained models you could use, see the [Tensorflow detection model zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md) for more.


**Other:**

env_pi.sh: Script to remote launch nodes on the raspberry pi from the jetson. This overcomes the need to ssh into the raspberry pi separately from the jetson to launch the camera node. This goes in ~/catkin_ws/devel/ on the raspberry pi.

roslocal.sh, rostb3.sh: Scripts to set your ROS IP settings.

CMakeLists.txt: Make file for the package


