#!/usr/bin/env python3

from parseCSVstring import *
import math

'''
This script will convert a .csv file of x,y coordinates into a .world file of a parking lot.

USAGE:
1)  Open sample_parking_lot.xls and edit the x,y coordinates.
2)  Export the spreadsheet as sample_parking_lot.csv  
3)  Open a terminal and change directories to where this script (create_parking_world.py) is saved.  Then:
	python3 create_parking_world.py
'''

# ----------------------------
# You might want to edit these
# ----------------------------
asphaltHeight = 0.01	# Increase to 0.02 if you want to hide the background grid from showing through the pavement.

lineWidth 	= 0.14
lineHeight 	= asphaltHeight + 0.01
lineColor 	= 'White'		# 'Yellow' should also work.

curbWidth	= 0.25		# "Thickness" of the curb
curbHeight	= asphaltHeight + 0.5	
curbColor	= 'Red'
# ----------------------------

# ----DON'T EDIT BELOW HERE---


class make_coords:
	def __init__(self, x, y, objLen, objAngle):
		# Sets the value of myCoords[counter_number]
		self.x 			= x
		self.y 			= y
		self.objLen 	= objLen 
		self.objAngle	= objAngle

# Initialize our data structures to store info for each object:
myLineCoords = {}
myCurbCoords = {}
myCarCoords  = {}

# Initialize our Counters
numLines = 0
numCurbs = 0
numCars  = 0

# Initialize our max/min coords
maxX = -float('Inf')
minX =  float('Inf')
maxY = -float('Inf')
minY =  float('Inf')

# Read from the .csv file:
dataFile = 'sample_parking_lot.csv'
rawData = parseCSVstring(dataFile, returnJagged=True, fillerValue=-1, delimiter=',', commentChar='%')
for i in range(0,len(rawData)):
	x 			= float(rawData[i][0])
	y 			= float(rawData[i][1])
	objType 	= str(rawData[i][2])
	objLen  	= float(rawData[i][3])
	objAngle	= float(rawData[i][4])

	maxX = max(maxX, x+objLen/2.0)
	minX = min(minX, x-objLen/2.0)
	maxY = max(maxY, y+objLen/2.0)
	minY = min(minY, y-objLen/2.0)
	
	if (objType == 'car'):
		numCars += 1
		myCarCoords[numCars] = make_coords(x, y, objLen, objAngle)		
	elif (objType == 'curb'):
		numCurbs += 1
		myCurbCoords[numCurbs] = make_coords(x, y, objLen, objAngle)			
	elif (objType == 'lane'):
		numLines += 1
		myLineCoords[numLines] = make_coords(x, y, objLen, objAngle)			
	else:
		print("Unknown Type in line %d: %s" % i+2, objType)
		
# Open the outfile for writing:
outFile = open("parking_lot.world",'w')

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

asphaltLength = maxX - minX + 1
asphaltWidth = maxY - minY + 1
asphaltX = (maxX + minX + 1) / 2.0
asphaltY = (maxY + minY + 1) / 2.0

asphaltInfo = """
    <model name='asphalt_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <box>
              <size>%f %f %f</size>
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
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>%f %f %f</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://asphalt_plane/materials/scripts</uri>
              <uri>model://asphalt_plane/materials/textures</uri>
              <name>vrc/asphalt</name>
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
      <pose>%f %f 0 0 -0 0</pose>
    </model>
""" % (asphaltLength, asphaltWidth, asphaltHeight, asphaltLength, asphaltWidth, asphaltHeight, asphaltX, asphaltY)
outFile.write(asphaltInfo)


# Create a model for each line:
for i in myLineCoords:
	thisString = """
	    <model name='line_%s'>
	      <pose>%f %f 0 0 -0 %f</pose>
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
	              <size>%f %f %f</size>
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
	              <size>%f %f %f</size>
	            </box>
	          </geometry>
	          <material>
	            <script>
	              <uri>file://media/materials/scripts/gazebo.material</uri>
	              <name>Gazebo/%s</name>
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
	""" % (str(i), myLineCoords[i].x, myLineCoords[i].y, myLineCoords[i].objAngle*(math.pi/180.0), myLineCoords[i].objLen, lineWidth, lineHeight, myLineCoords[i].objLen, lineWidth, lineHeight, lineColor)
	outFile.write(thisString)


# Create a model for each car:
for i in myCarCoords:
	thisString = """
	    <model name='pickup_%s'>
	      <static>1</static>
	      <link name='link'>
	        <collision name='collision'>
	          <pose>%f %f 0 0 0 %f</pose>
	          <geometry>
	            <mesh>
	              <uri>model://pickup/meshes/pickup.dae</uri>
	              <scale>1 1 1</scale>
	            </mesh>
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
	          <pose>%f %f 0 0 0 %f</pose>
	          <geometry>
	            <mesh>
	              <uri>model://pickup/meshes/pickup.dae</uri>
	            </mesh>
	          </geometry>
	        </visual>
	        <velocity_decay>
	          <linear>0</linear>
	          <angular>0</angular>
	        </velocity_decay>
	        <self_collide>0</self_collide>
	        <kinematic>0</kinematic>
	        <gravity>1</gravity>
	      </link>
	      <pose>0 0 0 0 -0 0</pose>
	    </model>
	""" % (str(i), myCarCoords[i].x, myCarCoords[i].y, myCarCoords[i].objAngle*(math.pi/180.0), myCarCoords[i].x, myCarCoords[i].y, myCarCoords[i].objAngle*(math.pi/180.0))
	outFile.write(thisString)


# Create a model for each Curb:
for i in myCurbCoords:
	thisString = """
	    <model name='curb_%s'>
	      <pose>%f %f 0 0 -0 %f</pose>
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
	              <size>%f %f %f</size>
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
	              <size>%f %f %f</size>
	            </box>
	          </geometry>
	          <material>
	            <script>
	              <uri>file://media/materials/scripts/gazebo.material</uri>
	              <name>Gazebo/%s</name>
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
	""" % (str(i), myCurbCoords[i].x, myCurbCoords[i].y, myCurbCoords[i].objAngle*(math.pi/180.0), myCurbCoords[i].objLen, curbWidth, curbHeight, myCurbCoords[i].objLen, curbWidth, curbHeight, curbColor)
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

print("See 'parking_lot.world'.")


