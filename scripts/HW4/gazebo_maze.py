import numpy as np
from maze_sim_parameters import MAZE

world_template = """
<sdf version='1.6'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose frame=''>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.5 -1</direction>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
      </link>
    </model>
{0}
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics name='default_physics' default='0' type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <state world_name='default'>
      <sim_time>0 0</sim_time>
      <real_time>0 0</real_time>
      <wall_time>1487236332 268614325</wall_time>
      <iterations>0</iterations>
      <model name='ground_plane'>
        <pose frame=''>0 0 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <light name='sun'>
        <pose frame=''>0 0 10 0 -0 0</pose>
      </light>
    </state>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose frame=''>5 -5 2 0 0.275643 2.35619</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
  </world>
</sdf>
"""

wall_template = """
    <model name='wall_{0}'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <box>
              <size>{1} {2} {3}</size>
            </box>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>{1} {2} {3}</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Orange</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
      </link>
      <pose frame=''>{4} {5} 0 0 0 {6}</pose>
    </model>
"""

WALL_HEIGHT = 1
WALL_WIDTH = 0.001

walls = []
for i, xy in enumerate(MAZE):
  x, y = np.array(xy[0]), np.array(xy[1])
  l = np.linalg.norm(y - x)
  c = (x + y)/2.0
  th = np.arctan2(y[1] - x[1], y[0] - x[0])
  walls.append(wall_template.format(i, l, WALL_WIDTH, WALL_HEIGHT, c[0], c[1], th))

with open("maze.world", "w") as f:    # put me in ~/catkin_ws/src/asl_turtlebot/world
  f.write(world_template.format("".join(walls)))