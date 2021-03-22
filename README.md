# winter_project
In this project I used 2 robots, ridgeback and sawyer arm. The arm is attached on the omnidirectionl mobile platform of ridgeback. The name of the final robot is sawback.
The goal of the project is to navigate autonomous in a room where tha map of it is already know. Detect objects (with AprilTags) and based on the detected object navigate and place it to its original positions. In this project we are able to grab some cubes and place them inside a box, on shelfs or open a drawer and place the cube inside.

![](https://github.com/jimas95/portfolio/blob/master/img/portfolio/sawback/sawback.png)



## Depended workspaces
The project extends 2 workspaces [sawback](https://github.com/jimas95/sawback) and [nu_ridgeback](https://github.com/jimas95/nu_ridgeback) both of them are forked from Boston Cleek and have some small changes. If you want to set the robot from the start you should read and follow the instruction steps from those two repositories. nu_ridgeback is for controling the mobile platform, while sawback uses nu_ridgback and contains packages for the sawyer arm. 

# Install workspace
1. clone this repo
2. extend it (link is with sawback)

# Execute project
After you have set the workspaces corectly you will need to set up the robot. Those steps are also mentioned at the previous mentioned repos, but I have them here in a more compact way. For more info check sawback and nu_ridgback repos.
1. `set the ROS_MASTER_URI of sawbacks IP `
2. `sudo ip route add 192.168.131.0/24 via 10.42.0.1`
3. `ssh administrator@192.168.131.1`
4. `systemctl stop ridgeback.service`
5. `consawyer`
6. `delrefs`
7. `source ~/jimas_ws/devel/setup.bash`
8. `rosrun intera_interface enable_robot.py -e`
9. `roslaunch commander launch_all.launch`
10. ` rosrun commander commander_node`
11. ` rosrun commander manipulation.py`
12. ` roslaunch nuridgeback_robot visualization.launch viz_lab:=true`

# Command 9
 `roslaunch commander launch_all.launch` 
 Will execute everything we need for the robots, such as activate sensors, switch cameras on, init MoveBase and MoveIT, rtabmap and PS4 controller.

# Command 10
` rosrun commander commander_node` 

Is a C++ node for controling the move_base action of the robot. This node initiates a sercive name `command` with a custom srv message.
Using he service you can set the ridgeback to idle state, patrol mode or set to go at a specific position at the string argument of the srv msg such as the labs kitchen, shelfs or charging station.


# Command 11
` rosrun commander manipulation.py`

This is the a python node where the mobile manipulation of grabing objects is done.

# Command 12
` roslaunch nuridgeback_robot visualization.launch viz_lab:=true`

Will open my rViz configuration.



# Note 
Everything should be executed in ridgeback computer. If you want to run them directly from your and not ssh to ridgeback use the specia argument of machine tag.

# Examples
## Pick and Place in a Box 

![](https://github.com/jimas95/portfolio/blob/master/img/portfolio/sawback/box.gif)
## Pick and Place at Shelfs

![](https://github.com/jimas95/portfolio/blob/master/img/portfolio/sawback/2OBJ_shelfs.gif)
## Pick and Place in a Drawer
![](https://github.com/jimas95/portfolio/blob/master/img/portfolio/sawback/drawer.gif)

