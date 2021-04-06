# multi-UAV-simulator
GNC of a SWARM of UAVs in a SITL by Tom Antoine and Alex Martinez - Cranfield University AVDC MSc 2021

This project is composed of two independant but similar repositories: 
- Simulation_Python : it is used to do numerical simulations on the guidance algorithm fully on Python (https://github.com/tomAntoine/multi-UAV-simulator.git)
- Simulation_Gazebo : here, the python code for guidance is used as SITL, for a Gazebo/ROS/Ardupilot world (below)

# Simulation_Gazebo
## Prerequisites:

Python 3.8.5 was employed, so full compability with that or after version
Basic Python libraries such as matplotlib, math, etc

Install Ubuntu (version 20.04 employed, but 18.04 should also work)

Install Gazebo and ArduPilot Plugin (https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_gazebo_arduplugin.md)

Install ArduPilot and MAVProxy (https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot_20_04.md)

Install ROS and Setup Catkin (noetic employed https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_ros_20_04.md, 
melodic should also work)

Repositories (iq_sim https://github.com/Intelligent-Quads/iq_sim)

Recommended:
QGroundControl 

All of them can be easily installed by following the tutorials below:
https://github.com/Intelligent-Quads/iq_tutorials

Make the workspace and build it each time there is a modification

## Main files

### run_3D_simulation.py

It is the main script from where the simulation is launched.  In the main, the scenario is selected and the quadcopters are initialized. 

Then the simulation is run by calling run_sim and the data can be visualized in plots or 3D animations. It also holds some of the scenarios created for V&V

### pf3d.py
It is the script for the Guidance algorithm, in this case the potential field in 3D. It can be easily exchanged by another algorithm. It returns discrete waypoints.

### quadFiles folder
#### quad.py
In order to implement the drones, both agents and enemies, the Quad class was created. This is a file 
that is used to firstly initialize the drones as independent objects and to assign and update their attributes. 

This method of implementation allowed a very easy and intuitive use of the code, as well as an unlimited scalability of the SWARM.  

For the initialization to happen several inputs need to be provided. The main ones are the drone 
identification number “quad_id”, the keyword “mode”, the target identification number “id_targ”, the goal 
position “pos_goal” and the position of static obstacles “pos_obs”. 

#### ROSquad.py
This library stores most of the functions employed to adapt the quad class file to the more complex environment. 
For that, the DroneKit library was initially taken, adapted and employed as an overlayer of MAVpy.

DroneKit, is an open-source python package that allows run scripts on an onboard companion computer and 
communicate with the ArduPilot flight controller using a low-latency link. DroneKit-Python can also be used 
for ground station apps, communicating with vehicles over a higher latency RF-link. Furthermore, it gives the
capability to connect many UAVs and give them different commands-navigation to several different waypoints 
along the simulation’s map by adjusting the communication channel to each of it. 

In order to communicate the algorithms that accomplished with the Arducopter, Mavlink communication protocol
was preferred. It is a messaging protocol for communicating with different agents. MAVLink follows a 
publish-subscribe and point-to-point design pattern: Data streams are as topics while configuration 
sub-protocols such as the mission protocol or parameter protocol are point-to-point with retransmission. 
Furthermore, MAVLink is compatible with Python, that is the language that was preferred to accomplish the 
algorithms. 

This allowed to use the MAVlink protocole by means of commands sent to the vehicle. However, the predefined 
functions from DroneKit were not enough and more functions were created to fully integrate the code.

 

### utils folder 
#### display.py
Creates the plots for the analysis

## Other files
### multi_drone.launch
Launch file that connects with Gazebo. It contains the quadcopters by calling their models and communications

### multi-ardupilot.sh
Launch file that connects with Arducopter. It opens a terminal for each arducopter

### multi-apm.launch
Launch file that connects with ROS.

### multi_drone.world
Contains the world with the drones. The environment can be changed in terms of objects (trees, buildings, quads) 
and initial position Main files

## How to launch the simulation
Before starting, the .sh need to be executable, open a terminal an run the following line for each filename.sh:

    chmode +x filename.sh

It will only need to be done once. Also, make sure the files in this folder replace the corresponding ones and that the directories match the ones 
below and do a catkin build before the first launch:

    # Check the directories of the files match
    ~/catkin_ws/src/iq_sim/launch/multi_drone.launch
    ~/multi-ardupilot.sh
    ~/catkin_ws/src/iq_sim/launch/multi-apm.launch
    ~/catkin_ws/src/iq_sim/worlds/multi_drone.world
    
    # Do a catkin build before the first launch
    ~/catkin_ws/src/catkin build


For every launch:

Open Terminal 1

    cd catkin_ws/src/
    roslaunch iq_sim multi_drone.launch

Open Terminal 2

    cd
    ./multi-ardupilot.sh

Open Terminal 3

    cd catkin_ws/src/
    roslaunch iq_sim multi-apm.launch

Open Terminal 4

    cd multi-UAV-simulator/
    python run_3D_simulation.py

