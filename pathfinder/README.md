# Create an autonomous pathfinder #
--------------------------
Turtlebot simulation in Gazebo. The aim of this task is to navigate the turtlebot autonomously from position (x,y) = (0,0) to (x,y) = (4,4) in a 9m x 9m map with a random arrangement of walls.


## Requrements ##
--------------------------
Robot Operating System (ROS) Kinetic distribution installed. 


## About the code ## 
-------------------------


## How to run the code ## 
-------------------------
1. Enter your source folder inside your catkin workspace:
	```bash
	$ cd catkin_ws/src
	```

2. Clone the repository: 
	```bash
	$ git clone https://github.com/Sollimann/ROS_tutorials.git
	```

3. Inside your catkin workspace, run:
	```bash
	$catkin_ws catkin build
	```

4. Run project\_init.sh or project\_init\_world_2.sh to lauch either world 1 or 2:
	```bash
	$ chmod +x project_init.sh
	$ ./project_init.sh
	```
or

	```bash
	$ chmod +x project_init_world_2.sh
	$ ./project_init_world_2.sh
	```

