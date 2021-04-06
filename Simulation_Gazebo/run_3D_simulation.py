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
import time
from quadFiles.quad import Quadcopter
import utils
import random 
import matplotlib.pyplot as plt

def quad_sim(Ts, quads):

    pos_quads = []

    for quad in quads:
        quad.update_states()

    for quad in quads:
        pos_quads.append([quad.quad_id,quad.pos])

    for quad in quads:

        quad.pos_quads = pos_quads

        if  quad.neutralize :

            quads[quad.id_targ].mode = "fall"

            quad.neutralize = False

            for quad_2 in quads:
                if quad_2.id_targ == quad.id_targ and quad_2 != quad:
                    quad_2.mode = "home"

        quad.update(Ts)

def main():

    Ts = 0.005 # final time of simulation
    Tc = time.time() # clock time of simulation

    # ---------------------------

    def gazebo_oa():
        pos_obs = []
        for i in range(30):
            pos_obs.append(random.sample(range(-20, 0), 3))
        pos_obs = np.array(pos_obs)
              
        quad0 = Quadcopter(Ts = Ts, quad_id = 0, mode='ennemy', id_targ = -1, pos_goal= [-20,-20,-20], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14551', global_frame=None)
        # Initialize the frame at the origin by initializing the drone located at the origin
        global_frame = quad0.global_frame
        #quad1 = Quadcopter(Ts = Ts, quad_id = 1, mode='ennemy', id_targ = -1, pos_goal= [5,-100,-15], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14561', global_frame=global_frame)
        #quad2 = Quadcopter(Ts = Ts, quad_id = 2, mode='ennemy', id_targ = -1, pos_goal= [-5,-100,-8], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14571', global_frame=global_frame)
        

        quads = [quad0] #, quad1, quad2]

        return pos_obs, quads



    def gazebo_ca():
        pos_obs = np.array([[-50,0,0]])
                      
        quad0 = Quadcopter(Ts = Ts, quad_id = 0, mode='guided', id_targ = -1, pos_goal= [120,0,-10], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14551', global_frame=None)
        # Initialize the frame at the origin by initializing the drone located at the origin
        global_frame = quad0.global_frame
        quad1 = Quadcopter(Ts = Ts, quad_id = 1, mode='ennemy', id_targ = -1, pos_goal= [-100,0,-10], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14561', global_frame=global_frame)
        quad2 = Quadcopter(Ts = Ts, quad_id = 2, mode='ennemy', id_targ = -1, pos_goal= [-100,0,-10], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14571', global_frame=global_frame)
        quad3 = Quadcopter(Ts = Ts, quad_id = 3, mode='ennemy', id_targ = -1, pos_goal= [-100,0,-10], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14581', global_frame=global_frame)
        #quad4 = Quadcopter(Ts = Ts, quad_id = 4, mode='ennemy', id_targ = -1, pos_goal= [-100,0,-10], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14591', global_frame=None)
        #quad5 = Quadcopter(Ts = Ts, quad_id = 5, mode='ennemy', id_targ = -1, pos_goal= [-100,0,-10], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14601', global_frame=None)
        #quad6 = Quadcopter(Ts = Ts, quad_id = 6, mode='ennemy', id_targ = -1, pos_goal= [-100,0,-10], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14611', global_frame=None)


        quads = [quad0, quad1, quad2, quad3]

        return pos_obs, quads


    def gazebo_track():
        pos_obs = np.array([[-50,0,0]])
                      
        quad0 = Quadcopter(Ts = Ts, quad_id = 0, mode='ennemy', id_targ = -1, pos_goal= [120,0,-10], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14551', global_frame=None)
        # Initialize the frame at the origin by initializing the drone located at the origin
        global_frame = quad0.global_frame
        quad1 = Quadcopter(Ts = Ts, quad_id = 1, mode='track', id_targ = 0, pos_goal= [0,0,-10], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14561', global_frame=global_frame)
        #quad2 = Quadcopter(Ts = Ts, quad_id = 2, mode='ennemy', id_targ = -1, pos_goal= [-100,0,-10], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14571', global_frame=global_frame)
        #quad3 = Quadcopter(Ts = Ts, quad_id = 3, mode='ennemy', id_targ = -1, pos_goal= [-100,0,-10], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14581', global_frame=global_frame)
        #quad4 = Quadcopter(Ts = Ts, quad_id = 4, mode='ennemy', id_targ = -1, pos_goal= [-100,0,-10], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14591', global_frame=None)
        #quad5 = Quadcopter(Ts = Ts, quad_id = 5, mode='ennemy', id_targ = -1, pos_goal= [-100,0,-10], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14601', global_frame=None)
        #quad6 = Quadcopter(Ts = Ts, quad_id = 6, mode='ennemy', id_targ = -1, pos_goal= [-100,0,-10], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14611', global_frame=None)


        quads = [quad0, quad1] #, quad2, quad3]

        return pos_obs, quads

    def simple_wp_cycle():

        pos_obs = np.array([1,1,-20])

        quad0 = Quadcopter(Ts = Ts, quad_id = 0, mode='ennemy', id_targ = -1, pos_goal= [10,-10,-10], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14551', global_frame=None)
        quads = [quad0]
        return pos_obs, quads


    pos_obs, quads = gazebo_track()
        

    for quad in quads:

        quad.init()

    tic = time.time()
    tac = time.time()
    while tac-tic<240 :


        quad_sim(Ts, quads)

        #time.sleep(3)
        tac = time.time()

    data = dict()

    t_all = np.array(quads[0].time_all)- tic
    t_all[0] = t_all[1]

    print(t_all)

    for quad in quads:
        if quad.mode_ini == 'ennemy':
            color = 'green'
        else:
            color = 'blue'
        data[str(quad.quad_id)] = dict([('mode',quad.mode_ini),
                                        ('id_targ',quad.id_targ),
                                        ('pos_goal', quad.pos_goal_ini),
                                        ('pos_all',np.array(quad.pos_all)),
                                        ('t_all',t_all),
                                        ('color',color)])



    utils.makeAllFigures(data, pos_obs)

    plt.show()



if __name__ == "__main__":
    print("simulation started")
    main()
