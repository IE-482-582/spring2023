#!/usr/bin/env python3

from parseCSVstring import *

'''
This script will convert a .csv file of x,y coordinates into a .world file containing a unit box located at each x,y coordinate.

USAGE:
1)  Open custom_world_coords.xls and edit the x,y coordinates.
2)  Export the spreadsheet as custom_world_coords.csv  
3)  Open a terminal and change directories to where this script (create_world.py) is saved.  Then:
	python3 create_world.py
'''

class make_coords:
	def __init__(self, x, y):
		# Sets the value of myCoords[box_number]
		self.x 	= x
		self.y 	= y

# Initialize our data structure to store our x,y coordinates for each obstacle:
myCoords = {}

# Read from the .csv file:
dataFile = 'custom_world_coords.csv'
rawData = parseCSVstring(dataFile, returnJagged=True, fillerValue=-1, delimiter=',', commentChar='%')
for i in range(0,len(rawData)):
	x = int(rawData[i][0])
	y = int(rawData[i][1])
	myCoords[i+1] = make_coords(x, y)
	
# Open the outfile for writing:
outFile = open("custom_world.world",'w')

preamble = """
<sdf version='1.4'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
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
            </friction>
            <bounce/>
            <contact>
              <ode/>
            </contact>
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
        <velocity_decay>
          <linear>0</linear>
          <angular>0</angular>
        </velocity_decay>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <gravity>1</gravity>
      </link>
    </model>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
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
"""
outFile.write(preamble)


# Create a model for each unit obstacle on the map:
for i in myCoords:
	thisString = """
	    <model name='unit_box_%s'>
	      <pose>%d %d 0 0 -0 0</pose>
	      <link name='link'>
	        <inertial>
	          <mass>1</mass>
	          <inertia>
	            <ixx>1</ixx>
	            <ixy>0</ixy>
	            <ixz>0</ixz>
	            <iyy>1</iyy>
	            <iyz>0</iyz>
	            <izz>1</izz>
	          </inertia>
	        </inertial>
	        <collision name='collision'>
	          <geometry>
	            <box>
	              <size>1 1 1</size>
	            </box>
	          </geometry>
	          <max_contacts>10</max_contacts>
	          <surface>
	            <bounce/>
	            <friction>
	              <ode/>
	            </friction>
	            <contact>
	              <ode/>
	            </contact>
	          </surface>
	        </collision>
	        <visual name='visual'>
	          <geometry>
	            <box>
	              <size>1 1 1</size>
	            </box>
	          </geometry>
	          <material>
	            <script>
	              <uri>file://media/materials/scripts/gazebo.material</uri>
	              <name>Gazebo/Grey</name>
	            </script>
	          </material>
	        </visual>
	        <velocity_decay>
	          <linear>0</linear>
	          <angular>0</angular>
	        </velocity_decay>
	        <self_collide>0</self_collide>
	        <kinematic>0</kinematic>
	        <gravity>1</gravity>
	      </link>
	      <static>1</static>
	    </model>
	""" % (str(i), myCoords[i].x, myCoords[i].y)
	outFile.write(thisString)


stateOpening = """
    <state world_name='default'>
      <sim_time>39 388000000</sim_time>
      <real_time>39 522101720</real_time>
      <wall_time>1512656316 460847358</wall_time>
      <model name='ground_plane'>
        <pose>0 0 0 0 -0 0</pose>
        <link name='link'>
          <pose>0 0 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
"""
outFile.write(stateOpening)

# Create a state for each model:
for i in myCoords:
	thisString = """
	      <model name='unit_box_%s'>
	        <pose>%d %d 0 0 -0 0</pose>
	        <link name='link'>
	          <pose>%d %d 0 0 -0 0</pose>
	          <velocity>0 0 0 0 -0 0</velocity>
	          <acceleration>0 0 0 0 -0 0</acceleration>
	          <wrench>0 0 0 0 -0 0</wrench>
	        </link>
	      </model>
	""" % (str(i), myCoords[i].x, myCoords[i].y, myCoords[i].x, myCoords[i].y)
	outFile.write(thisString)

stateClosing = """
    </state>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>5 -5 2 0 0.275643 2.35619</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>
  </world>
</sdf>
"""
outFile.write(stateClosing)

outFile.close()

print("The file 'custom_world.world' has been generated.")


