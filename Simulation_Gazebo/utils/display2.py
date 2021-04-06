# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
import utils

rad2deg = 180.0/pi
deg2rad = pi/180.0
rads2rpm = 60.0/(2.0*pi)
rpm2rads = 2.0*pi/60.0

# Print complete vector or matrices
def fullprint(*args, **kwargs):
    opt = np.get_printoptions()
    np.set_printoptions(threshold=np.inf)
    print(*args, **kwargs)
    np.set_printoptions(opt)

def dist(a,b):
    return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2+(a[2]-b[2])**2)

def makeAllFigures(data, pos_obs):

    n = len(data[id_quad]["pos_all"])
    time = np.linspace(0,20,n)
    #new_lines
    id_list = []
    # Waypoint accuracy

    for id_quad in data:
        id_list.append(id_quad)

    # Waypoint accuracy

    for id_quad in (data):

        pos = data[id_quad]["pos_all"]
        dist2way_id = []

        for i in range(len(pos)):
            way = data[id_quad]["pos_goal"]
            dist2way_id.append(dist(way,pos[i,:]))

        data[id_quad]['dist2way'] = dist2way_id
       
    '''  
    # Obstacle avoidance

    for id_quad in (data):

        pos = data[id_quad]["pos_all"]
        dist2obs_id = []

        for i in range(len(pos)):

            dist2obs_i = []
            for obs in pos_obs :

                dist2obs_i.append(dist(obs,pos[i,:]))

            dist2obs_id.append(min(dist2obs_i))

        data[id_quad]['dist2obs'] = dist2obs_id
    '''
    
    # Collision avoidance

    for id_quad in (data):

        pos = data[id_quad]["pos_all"]
        dist2agt_id = []

        for i in range(len(pos)):

            dist2agt_i = []

            for id_quad_2 in (data):

                if (id_quad != id_quad_2) and (id_quad_2 != data[id_quad]["id_targ"]) :

                    pos_2 = data[id_quad_2]["pos_all"]

                    dist2agt_i.append(dist(pos_2[i,:],pos[i,:]))

            dist2agt_id.append(min(dist2agt_i))

        data[id_quad]['dist2agt'] = dist2agt_id
    
    '''
    # Traking performance

    id_trackers = []

    for id_quad in (data):

        if (data[id_quad]["mode"] == "track") :

            id_trackers.append(id_quad)            

            pos = data[id_quad]["pos_all"]
            id_target = str(data[id_quad]["id_targ"])
            pos_2 = data[id_target]["pos_all"]

            dist2target = []

            for i in range(len(pos)):

                dist2target.append(dist(pos[i,:],pos_2[i,:]))

            data[id_quad]['dist2target'] = dist2target
    '''


    ### plots

    plt.show()

    plt.figure(figsize = (10,10))
    for id_quad in (data):
        # Prepare the legend by agents and ennemies depending on modes
        if data[id_quad]["mode"] == 'track':
            sentence = 'Quad ID: {}, MODE: {}, TARGET: Quad {}'.format(id_quad,data[id_quad]["mode"],data[id_quad]["id_targ"])
        elif data[id_quad]["mode"] != 'track':
            sentence = 'Quad ID: {}, MODE: {}'.format(id_quad,data[id_quad]["mode"])
        id_plot = plt.plot(time, data[id_quad]["dist2way"], color = data[id_quad]["color"], label = sentence)
    min_dist = 100
    for id_quad in (data):
        min_dis_i = min(data[id_quad]["dist2way"])
        if min_dis_i < min_dist:
            min_dist = min_dis_i
    sentence = 'Final Distance to Waypoint {} m'.format(round(min_dist,3))
    min_dist_obj = plt.plot(time,[min_dist for t in time], '--', color = 'red', label = sentence)
    plt.grid(True)
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.title('Distance to Waypoint')    
    plt.draw()
    plt.savefig('Sim0_wp.png', dpi=120)

    '''
    plt.figure(figsize = (10,10))
    for id_quad in (data):
        # Prepare the legend by agents and ennemies depending on modes
        if data[id_quad]["mode"] == 'track':
            sentence = 'Quad ID: {}, MODE: {}, TARGET: Quad {}'.format(id_quad,data[id_quad]["mode"],data[id_quad]["id_targ"])
        elif data[id_quad]["mode"] != 'track':
            sentence = 'Quad ID: {}, MODE: {}'.format(id_quad,data[id_quad]["mode"])
        id_plot = plt.plot(time, data[id_quad]["dist2obs"], color = data[id_quad]["color"], label = sentence)
    min_dist = 100
    for id_quad in (data):
        min_dis_i = min(data[id_quad]["dist2obs"])
        if min_dis_i < min_dist:
            min_dist = min_dis_i
    sentence = 'Minimum distance {} m'.format(round(min_dist,3))
    min_dist_obj = plt.plot(time,[min_dist for t in time], '--', color = 'red', label = sentence)
    plt.grid(True)
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.title('Minimum Distance to Obstacle')    
    plt.draw()
    plt.savefig('Gazebo/Sim0_oa.png', dpi=120)
    
 
    '''
    plt.figure(figsize = (10,10))
    for id_quad in (data):
        # Prepare the legend by agents and ennemies depending on modes
        if data[id_quad]["mode"] == 'track':
            sentence = 'Quad ID: {}, MODE: {}, TARGET: Quad {}'.format(id_quad,data[id_quad]["mode"],data[id_quad]["id_targ"])
        elif data[id_quad]["mode"] != 'track':
            sentence = 'Quad ID: {}, MODE: {}'.format(id_quad,data[id_quad]["mode"])
        id_plot = plt.plot(time, data[id_quad]["dist2agt"], color = data[id_quad]["color"], label = sentence)
    min_dist = 100
    for id_quad in (data):
        min_dis_i = min(data[id_quad]["dist2agt"])
        if min_dis_i < min_dist:
            min_dist = min_dis_i
    sentence = 'Minimum distance {} m'.format(round(min_dist,3))
    min_dist_obj = plt.plot(time,[min_dist for t in time], '--', color = 'red', label = sentence)
    plt.grid(True)
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.title('Minimum Distance to Other Agents')    
    plt.draw()
    plt.savefig('Sim0_ca.png', dpi=120)
    
    

    '''

    if id_trackers :

        plt.figure()
        for id_quad in (id_trackers):
            # Prepare the legend by agents and ennemies depending on modes
            sentence = 'Quad ID: {}, MODE: {}, TARGET: Quad {}'.format(id_quad,data[id_quad]["mode"],data[id_quad]["id_targ"])
            id_plot = plt.plot(time, data[id_quad]["dist2target"], color = data[id_quad]["color"], label = sentence)
        plt.grid(True)
        sentence = 'Threshold 2 m'
        threshold = plt.plot(time,[1 for t in time], '--', color = 'red', label = sentence)
        plt.legend()
        plt.xlabel('Time (s)')
        plt.ylabel('Distance (m)')
        plt.title('Minimum Distance to Target')    
        plt.draw()
         #plt.savefig('ObstacleAvoidance/Sim5_oa.png', dpi=80)

    '''

def makeFigures(params, time, pos_all, vel_all, quat_all, omega_all, euler_all, commands, wMotor_all, thrust, torque, sDes_traj, sDes_calc):
    x    = pos_all[:,0]
    y    = pos_all[:,1]
    z    = pos_all[:,2]
    q0   = quat_all[:,0]
    q1   =  quat_all[:,1]
    q2   = quat_all[:,2]
    q3   = quat_all[:,3]
    xdot = vel_all[:,0]
    ydot = vel_all[:,1]
    zdot = vel_all[:,2]
    p    = omega_all[:,0]*rad2deg
    q    = omega_all[:,1]*rad2deg
    r    = omega_all[:,2]*rad2deg

    wM1  = wMotor_all[:,0]*rads2rpm
    wM2  = wMotor_all[:,1]*rads2rpm
    wM3  = wMotor_all[:,2]*rads2rpm
    wM4  = wMotor_all[:,3]*rads2rpm

    phi   = euler_all[:,0]*rad2deg
    theta = euler_all[:,1]*rad2deg
    psi   = euler_all[:,2]*rad2deg

    x_sp  = sDes_calc[:,0]
    y_sp  = sDes_calc[:,1]
    z_sp  = sDes_calc[:,2]
    Vx_sp = sDes_calc[:,3]
    Vy_sp = sDes_calc[:,4]
    Vz_sp = sDes_calc[:,5]
    x_thr_sp = sDes_calc[:,6]
    y_thr_sp = sDes_calc[:,7]
    z_thr_sp = sDes_calc[:,8]
    q0Des = sDes_calc[:,9]
    q1Des = sDes_calc[:,10]
    q2Des = sDes_calc[:,11]
    q3Des = sDes_calc[:,12]    
    pDes  = sDes_calc[:,13]*rad2deg
    qDes  = sDes_calc[:,14]*rad2deg
    rDes  = sDes_calc[:,15]*rad2deg

    x_tr  = sDes_traj[:,0]
    y_tr  = sDes_traj[:,1]
    z_tr  = sDes_traj[:,2]
    Vx_tr = sDes_traj[:,3]
    Vy_tr = sDes_traj[:,4]
    Vz_tr = sDes_traj[:,5]
    Ax_tr = sDes_traj[:,6]
    Ay_tr = sDes_traj[:,7]
    Az_tr = sDes_traj[:,8]
    yaw_tr = sDes_traj[:,14]*rad2deg

    uM1 = commands[:,0]*rads2rpm
    uM2 = commands[:,1]*rads2rpm
    uM3 = commands[:,2]*rads2rpm
    uM4 = commands[:,3]*rads2rpm

    x_err = x_sp - x
    y_err = y_sp - y
    z_err = z_sp - z

    psiDes   = np.zeros(q0Des.shape[0])
    thetaDes = np.zeros(q0Des.shape[0])
    phiDes   = np.zeros(q0Des.shape[0])
    for ii in range(q0Des.shape[0]):
        YPR = utils.quatToYPR_ZYX(sDes_calc[ii,9:13])
        psiDes[ii]   = YPR[0]*rad2deg
        thetaDes[ii] = YPR[1]*rad2deg
        phiDes[ii]   = YPR[2]*rad2deg
    
    plt.show()

    plt.figure()
    plt.plot(time, x, time, y, time, z)
    plt.plot(time, x_sp, '--', time, y_sp, '--', time, z_sp, '--')
    plt.grid(True)
    plt.legend(['x','y','z','x_sp','y_sp','z_sp'])
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.draw()



    plt.figure()
    plt.plot(time, xdot, time, ydot, time, zdot)
    plt.plot(time, Vx_sp, '--', time, Vy_sp, '--', time, Vz_sp, '--')
    plt.grid(True)
    plt.legend(['Vx','Vy','Vz','Vx_sp','Vy_sp','Vz_sp'])
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.draw()

    plt.figure()
    plt.plot(time, x_thr_sp, time, y_thr_sp, time, z_thr_sp)
    plt.grid(True)
    plt.legend(['x_thr_sp','y_thr_sp','z_thr_sp'])
    plt.xlabel('Time (s)')
    plt.ylabel('Desired Thrust (N)')
    plt.draw()

    plt.figure()
    plt.plot(time, phi, time, theta, time, psi)
    plt.plot(time, phiDes, '--', time, thetaDes, '--', time, psiDes, '--')
    plt.plot(time, yaw_tr, '-.')
    plt.grid(True)
    plt.legend(['roll','pitch','yaw','roll_sp','pitch_sp','yaw_sp','yaw_tr'])
    plt.xlabel('Time (s)')
    plt.ylabel('Euler Angle (°)')
    plt.draw()
    
    plt.figure()
    plt.plot(time, p, time, q, time, r)
    plt.plot(time, pDes, '--', time, qDes, '--', time, rDes, '--')
    plt.grid(True)
    plt.legend(['p','q','r','p_sp','q_sp','r_sp'])
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (°/s)')
    plt.draw()

    plt.figure()
    plt.plot(time, wM1, time, wM2, time, wM3, time, wM4)
    plt.plot(time, uM1, '--', time, uM2, '--', time, uM3, '--', time, uM4, '--')
    plt.grid(True)
    plt.legend(['w1','w2','w3','w4'])
    plt.xlabel('Time (s)')
    plt.ylabel('Motor Angular Velocity (RPM)')
    plt.draw()

    plt.figure()
    plt.subplot(2,1,1)
    plt.plot(time, thrust[:,0], time, thrust[:,1], time, thrust[:,2], time, thrust[:,3])
    plt.grid(True)
    plt.legend(['thr1','thr2','thr3','thr4'], loc='upper right')
    plt.xlabel('Time (s)')
    plt.ylabel('Rotor Thrust (N)')
    plt.draw()

    plt.subplot(2,1,2)
    plt.plot(time, torque[:,0], time, torque[:,1], time, torque[:,2], time, torque[:,3])
    plt.grid(True)
    plt.legend(['tor1','tor2','tor3','tor4'], loc='upper right')
    plt.xlabel('Time (s)')
    plt.ylabel('Rotor Torque (N*m)')
    plt.draw()

    plt.figure()
    plt.subplot(3,1,1)
    plt.title('Trajectory Setpoints')
    plt.plot(time, x_tr, time, y_tr, time, z_tr)
    plt.grid(True)
    plt.legend(['x','y','z'], loc='upper right')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')

    plt.subplot(3,1,2)
    plt.plot(time, Vx_tr, time, Vy_tr, time, Vz_tr)
    plt.grid(True)
    plt.legend(['Vx','Vy','Vz'], loc='upper right')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
   
    plt.subplot(3,1,3)
    plt.plot(time, Ax_tr, time, Ay_tr, time, Az_tr)
    plt.grid(True)
    plt.legend(['Ax','Ay','Az'], loc='upper right')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s^2)')
    plt.draw()

    plt.figure()
    plt.plot(time, x_err, time, y_err, time, z_err)
    plt.grid(True)
    plt.legend(['Pos x error','Pos y error','Pos z error'])
    plt.xlabel('Time (s)')
    plt.ylabel('Position Error (m)')
    plt.draw()