# -*- coding: utf-8 -*-
"""
Adapted by:
Tom Antoine and Alejandra Martínez
part of GNC subteam, group 1, GDP AVDC 2020-2021
email:
tom.antoine@cranfield.ac.uk
alejandra.martinez-farina@cranfield.ac.uk

Based on a code by:
author: John Bass
email: john.bobzwik@gmail.com
github: https://github.com/bobzwik/Quadcopter_SimCon
license: MIT


Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from numpy import sin, cos, tan, pi, sign
from scipy.integrate import ode

import utils
import config

from pf3d import pf3d

from quadFiles.ROSQuad import *

import time

deg2rad = pi/180.0


class Quadcopter:
    '''
    In order to implement the drones, both agents and enemies, the Quad class was created. 
    This is a file that is used to firstly initialize the drones as independent objects and 
    to assign and update their attributes. 

    This method of implementation allowed a very easy and intuitive use of the code, as well
    as an unlimited scalability of the SWARM.  

    For the initialization to happen several inputs need to be provided. The main ones are the 
    drone identification number “quad_id”, the keyword “mode”, the target identification number 
    “id_targ”, the goal position “pos_goal” and the position of static obstacles “pos_obs”. 

    '''

    def __init__(self, Ts, quad_id = None, mode = 'ennemy', id_targ = -1, pos_goal= [1,1,-1], pos_obs = None, channel_id = None, global_frame = None):

        '''
        It is used to store in the basic attributes the inputs already mentioned in the previous sections. 

        In the case of the one employed for the more complex environment, the communication is also initialized 
        here. For that, there needs to be an association between the quad class in python and the vehicle from 
        Arducopter. This is done through the connect function that inputs the channel identification code and 
        returns the associated vehicle entity. Moreover, for the first drone, the global frame will also be 
        initialized and employed as reference for the others. 

        '''
        # Quad Params
        # ---------------------------

        self.quad_id = quad_id

        self.time_all = []


        self.pos_goal = pos_goal
        self.pos_goal_ini = pos_goal
        self.pos_obs = pos_obs
        self.pos_all = []
        self.id_targ = id_targ

        self.pos_quads = None

        self.mode = mode
        self.mode_ini = mode
        self.neutralize = False

        self.Ts = Ts
        self.t_track = 0

        self.previous_pos_targ = pos_goal
        #-- Setup the commanded flying speed
        self.gnd_speed = 4 # [m/s]

        self.channel_id = channel_id


        #-- Connect to the vehicle
        print('Connecting the drone {0}'.format(self.channel_id))

        self.vehicle = connect(channel_id)

        if not global_frame:
            self.global_frame = self.vehicle.location.global_relative_frame
        else: 
            self.global_frame = global_frame

        self.pos_ini = reverse_get_location_metres(self.global_frame, self.vehicle.location.global_relative_frame)

        self.vehicle.groundspeed = self.gnd_speed

    def update(self, Ts):
        '''
        It is used to update the attributes of each drone. First of all, the Boolean must be changed into true for
        specific modes and to false for others following their nature. 

        If the initial mode is “enemy”, no update is required as neither collision avoidance or tracking features 
        are activated. However, in the python environment, if the target becomes neutralized and its mode is changed
        into “fall”, the goal position will be updated once as its initial goal position with null height to simulate 
        its crash. This is done by the MAVlink command “stabilize” in the more complex environment.

        If the initial mode is not “enemy” and the Boolean is true, then update occurs. In the case of an agent in “guided”
        mode, it will update the position of other drones as dynamic obstacles to enhance collision avoidance. For that, it
        will compare the identification number of each drone and verify it does not match its own. Then, the temporal position 
        of obstacles will be jointly stored containing the static and dynamic ones. After that, the guidance algorithm will 
        be called and the trajectory updated. For an automated transition of modes, a distance threshold can be employed to 
        consider the goal position reached. For that, the norm of the difference between the current position and the goal one 
        is computed and compared with said threshold. If it is smaller, the mode can be switched to “home”.

        In the case of an agent in “track” mode, it will update both the position of other drones for collision avoidance and 
        its goal position. For that, the collision avoidance happens similarly to “guided” mode although in this case, it will 
        discard the identification numbers that match both its own and the target it is tasked to follow. As for the tracking aspect, 
        the target position is assumed to be provided by the Situation Awareness team. However, in the initial stages it is extracted
        from the environment. For a basic tracking, the goal position is assumed to be the target position with an additional half 
        meter in z axis, in order for the agent to hover over it. Nonetheless, this means that the agent will never be able to hover 
        exactly over the target. For an improved tracking performance, the future position of the target is estimated using a 
        constant velocity assumption. This discretized velocity is computed based on its current and past positions. This estimation
        can be further improved by including the continuous velocity, acceleration or by tuning its parameters. This estimation, 
        after adding the hovering distance is employed for the path planning. In order to check whether or not a target has been 
        neutralized two thresholds are needed, one associated with time and another one with the distance between the tracking agent
        and its target. In this project, the thresholds are set to 1m for 1.5s (three times the update time). If both are met, the 
        Boolean neutralize becomes true. This one will later be used to change the target mode from “enemy” to “neutralized”. Once 
        again, a normal automated transition for the agent would be “home” mode. 

        In the case of the remaining modes associated with actions, the longer ones can also include collision avoidance as a feature
        and therefore require updating. This is the case of “home”, “land” or “hover”, but not the case of “charging” or “takeoff”. 
        Nonetheless, in the more complex environment, most of these are predefined and can be directly sent as MAVlink commands.

        After the mode and goal position are updated, the trajectories are overwritten. For that, the current states are updated and
        the commands recomputed.

        '''


        self.update_states()

        if self.mode == "fall":

            #self.pos_goal = np.hstack([self.pos[0], self.pos[1], -0.5]).astype(float)
            ChangeMode(self.vehicle,"STABILIZE")
            
            self.mode = "neutralized"

            self.print_mode("ennemy", "fall")


        if (self.mode != 'ennemy') :
            if self.mode == 'track':
                pos_dyn_obs = [x[1] for x in self.pos_quads if  (x[0] != self.id_targ and x[0] != self.quad_id)]
                try :
                    temp_pos_obs = np.vstack((self.pos_obs, pos_dyn_obs)).astype(float)
                except :
                    temp_pos_obs = np.vstack((self.pos_obs)).astype(float)
                pos_targ = [x[1] for x in self.pos_quads if x[0] == self.id_targ][0]

                # mix desired velocity vector with the velocity of the target
                # lectures on coop guidance P16 ==> modif for moving target (it will add a gain)
                # need to understand the guidance command we are using
                # derivative ==> more reactivness
                # otherwise moddif wp but stab issues
                # ask SA if they can provide us the velocity of the targets, better than discretize estimation
                print(pos_targ)
                print(self.previous_pos_targ)
                estimate_pos_targ = [0,0,0]
                estimate_pos_targ[0] = 2*pos_targ[0]-self.previous_pos_targ[0]
                estimate_pos_targ[1] = 2*pos_targ[1]-self.previous_pos_targ[1]
                estimate_pos_targ[2] = 2*pos_targ[2]-self.previous_pos_targ[2]

                self.pos_goal = np.hstack((estimate_pos_targ) + [0,0,-1.5]).astype(float)
                dist = np.sqrt((self.pos[0]-self.pos_goal[0])**2+(self.pos[1]-self.pos_goal[1])**2+(self.pos[2]-self.pos_goal[2])**2)
                print('distance = {0}'.format(dist))
                if dist < 10:
                    self.t_track += Ts
                    if self.t_track > 3*Ts:
                        print(self.t_track)
                        self.neutralize = True
                        self.mode = "home"
                        self.print_mode("track", "home")
                else :
                    self.t_track = 0
                self.previous_pos_targ = pos_targ


            if self.mode == 'guided':
                pos_dyn_obs = [x[1] for x in self.pos_quads if  (x[0] != self.quad_id)]
                try :
                    temp_pos_obs = np.vstack((self.pos_obs, pos_dyn_obs)).astype(float)
                except :
                    temp_pos_obs = np.vstack((self.pos_obs)).astype(float)

                dist = np.sqrt((self.pos[0]-self.pos_goal[0])**2+(self.pos[1]-self.pos_goal[1])**2+(self.pos[2]-self.pos_goal[2])**2)
                if dist < 1:
                    self.mode = "home"
                    self.print_mode("guided", "home")

            if self.mode == 'home':
            	ChangeMode(self.vehicle,"RTL")
            	# be able to nkow when RTL is done 
            	if not vehicle.armed :
            		print("back home")

            if self.mode == 'takeoff': 
            	arm_and_takeoff(self.vehicle,3) 
            	print("takeoff finished")
            	
            if self.mode == "charging":
            	self.vehicle.battery.level += 10 
            	if self.vehicle.battery.level == 100: 
            		print('charged')


            if self.mode == 'land':
                pos_dyn_obs = [x[1] for x in self.pos_quads if  (x[0] != self.quad_id)]
                try :
                    temp_pos_obs = np.vstack((self.pos_obs, pos_dyn_obs)).astype(float)
                except :
                    temp_pos_obs = np.vstack((self.pos_obs)).astype(float)
                self.pos_goal = np.hstack([self.pos[0], self.pos[1], -0.5]).astype(float)

            if self.mode == 'hover':
                try :
                    temp_pos_obs = np.vstack((self.pos_obs, pos_dyn_obs)).astype(float)
                except :
                    temp_pos_obs = np.vstack((self.pos_obs)).astype(float)
                temp_pos_obs = np.vstack((self.pos_obs, pos_dyn_obs)).astype(float)
                self.pos_goal = np.hstack(self.pos).astype(float)


            self.wps, data = pf3d(self.pos, self.pos_goal, self.pos_obs)
            self.data = data
            #self.wps = [self.pos_goal]
            adds_new_mission(self.vehicle, self.vehicle.location.global_frame, self.wps, spline = False)

    def init(self):

        '''
        It is employed to input the first value of the attributes after initialization in the python environment. 
        
        However, in the more complex environment, this step is more difficult to carry out. It is required to first 
        arm and takeoff the vehicle, then to initiate the waypoints with the goal position. This allows to add a new
        mission and overwrite the initial state.

        '''

        arm_and_takeoff(self.vehicle,3)
        self.update_states()
        self.wps, _ = pf3d(self.pos, self.pos_goal, self.pos_obs)
        #self.wps = [self.pos_goal]
        adds_new_mission(self.vehicle, self.vehicle.location.global_frame, self.wps, spline = False)
        
        ChangeMode(self.vehicle,"AUTO")
        #print (self.vehicle.parameters['ANGLE_MAX'])
        self.vehicle.parameters['ANGLE_MAX']= 8000       

    def update_states(self):

        '''
        Only for the complex environment. It updates the states of the drones from the environment data by means of functions in the ROS quad library.
        '''

        self.pos = reverse_get_location_metres(self.global_frame, self.vehicle.location.global_relative_frame)
        pos_z_inv = [self.pos[0],self.pos[1],-self.pos[2]]
        self.pos_all.append(pos_z_inv)
        self.speed = self.vehicle.velocity
        self.time_all.append(time.time())
        print('_'*30)
        print(self.quad_id)
        print(pos_z_inv)
        print(self.speed)

    '''
    def print_mode(self, mode_prev, mode_new):
        print("Drone {0} switch from {1} to {2}.".format(self.quad_id, mode_prev, mode_new))


    def updateWaypoints2ROS(self):
        #clear_all_mission(self.vehicle)
        N_wps_before, _ = get_current_mission(self.vehicle)

        new_wps = self.wps
        print("new wps are : ----------------")
        for i in range(10):
            wp = new_wps[i+1]
            x, y, z = wp[0], wp[1], -wp[2]
            wp = get_location_metres(self.global_frame,x,y,z)
            print(x,y,z)
            add_last_waypoint_to_mission(self.vehicle, wp.lat, wp.lon, wp.alt)
        add_last_waypoint_to_mission(self.vehicle, wp.lat, wp.lon, wp.alt)
        print(N_wps_before)
        self.vehicle.commands.next = N_wps_before
        ChangeMode(self.vehicle,"AUTO")

    '''




