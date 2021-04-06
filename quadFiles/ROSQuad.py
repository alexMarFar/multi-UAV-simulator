"""
Adapted by:
# Adapted by: Alejandra Martínez, Charalampos Efstratiou and Tom Antoine
# GNC subteam, group 1, GDP AVDC 2020-2021
# email:
# alejandra.martinez-farina@cranfield.ac.uk
# charalampos.efstratiou@cranfield.ac.uk
# tom.antoine@cranfield.ac.uk

Based on DroneKit


Please feel free to use and modify this, but keep the above information. Thanks!
"""

import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import numpy as np
import math

'''
This library stores most of the functions employed to adapt the quad class file to the more
complex environment. For that, the DroneKit library was initially taken, adapted and employed
as an overlayer of MAVpy.

DroneKit, is an open-source python package that allows run scripts on an onboard companion
computer and communicate with the ArduPilot flight controller using a low-latency link. DroneKit
-Python can also be used for ground station apps, communicating with vehicles over a higher
latency RF-link. Furthermore, it gives the capability to connect many UAVs and give them different
commands-navigation to several different waypoints along the simulation’s map by adjusting the
communication channel to each of it.

In order to communicate the algorithms that accomplished with the Arducopter, Mavlink communication
protocol was preferred. It is a messaging protocol for communicating with different agents. MAVLink
follows a publish-subscribe and point-to-point design pattern: Data streams are as topics while
configuration sub-protocols such as the mission protocol or parameter protocol are point-to-point
with retransmission. Furthermore, MAVLink is compatible with Python, that is the language that was
preferred to accomplish the algorithms.

This allowed to use the MAVlink protocole by means of commands sent to the vehicle. However, the
predefined functions from DroneKit were not enough and more functions were created to fully integrate
the code.

'''


#--------------------------------------------------
#-------------- FUNCTIONS
#--------------------------------------------------
#-- Define arm and takeoff
def arm_and_takeoff(vehicle,altitude):
    '''
    A basic function from DroneKit that checks whether or not the vehicle is armable. Once it becomes
    armable, it is armed and commanded to take off at a given altitude. This altitude corresponds to
    the goal position provided by the Mission planning and Task allocation subsystem. Once the altitude
     has been reached, a message is sent and the drone is put in standby.
    '''

    while not vehicle.is_armable:
        print("waiting to be armable")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed: time.sleep(1)

    print("Taking Off")
    vehicle.simple_takeoff(altitude)

    while True:
        v_alt = vehicle.location.global_relative_frame.alt
        print(">> Altitude = %.1f m"%v_alt)
        if v_alt >= altitude - 0.1:
            print("Target altitude reached")
            break
        time.sleep(1)




def clear_all_mission(vehicle):
    '''
    A basic function from DroneKit that clears the previous mission commands. It is essential to
    clear past commands to be able to send new ones to the vehicle.
    '''
    vehicle.commands.clear()


def download_mission(vehicle):
    """
    A basic function from DroneKit that downloads previous mission commands.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.


def get_current_mission(vehicle):
    """
    After downloading the mission, it counts and returns the number of commands in the current
    mission. It also returns the commands in a list.

    Input:
        vehicle

    Return:
        n_wp, wpList
    """

    print ("Downloading mission")
    download_mission(vehicle)
    missionList = []
    n_WP        = 0
    for wp in vehicle.commands:
        missionList.append(wp)
        n_WP += 1

    return n_WP, missionList



def ChangeMode(vehicle, mode):
    '''
    A basic function from DroneKit that changes the mode of the vehicle.
    '''
    while vehicle.mode != VehicleMode(mode):
            print(mode)
            vehicle.mode = VehicleMode(mode)
            time.sleep(0.5)
    return True


def get_location_metres(original_location, dNorth, dEast, altitude):
    """
    A basic function from DroneKit that returns the GPS coordinates in the global reference frame
    from the original x, y and z coordinates. For that it assumes an spherical Earth radius of
    6378.137km and a flat Earth approximation. It is call each time a new mission is added, in
    order to transform the output waypoints from the guidance algorithm into GPS coordinate and
    altitude.



    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres
    from the specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    newalt = original_location.alt + altitude
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,newalt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,newalt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation;

def reverse_get_location_metres(original_location, location):
    '''
    A function that inverses the work on get_location_metres function. It is designed to provide x, y
    and z coordinates. It is called during initialization to write the original GPS coordinates from the
    environment in the desired format. It is  computed with respect to the global frame initialized by
    the first drone (the one at the origin of the simulation environment). It is also employed during
    the updates.
    '''

    lat = location.lat
    lon = location.lon
    alt = location.alt

    earth_radius = 6378137.0

    dLat = (lat - original_location.lat)*math.pi/180
    dLon = (lon - original_location.lon)*math.pi/180

    dNorth = dLat*earth_radius
    dEast  = dLon*(earth_radius*math.cos(math.pi*original_location.lat/180))

    return(dNorth,dEast,alt)


def adds_new_mission(vehicle, global_frame, wps_in_meter, spline = False):
    """
    A function that uplinks the next set of commands into a mission. For that, it initially extracts
    and clears the previous commands from the vehicle. Then, it adds the new ones following the MAVlink
    protocole. For a good integration of the guidance algorithm, this function calls get_location_metres
    to convert the waypoints into the correct format. For a correct functioning, an extra waypoint needs
    to be added. Finally, the new mission is uploaded to the vehicle and the command counter is reset
    to 0 so that the drone will fly to the first waypoint.


    Adds a takeoff command and waypoints commands to the current mission.
    The function assumes vehicle.commands matches the vehicle mission state
    (you must have called download at least once in the session and after clearing the mission)
    """
    if spline:
        trajType = mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT
    else:
        trajType = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT


    cmds = vehicle.commands

    #print(" Clear any existing commands")
    cmds.clear()

    #print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class.

    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    '''
    A basic function from DroneKit that is employed to command the vehicle by means of the MAVlink
    protocole. It has a set of 14 inputs that depend on the type of command. For this project, only
    the navigation to waypoints is employed. Its main inputs are the frame employed, the type of
    trajectory desired, and the waypoint in latitude, longitude, and altitude format.
    '''
    cmds.add(Command( 0, 0, 0, 3, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    points = []
    for wp in wps_in_meter:
        x = wp[0]
        y = wp[1]
        z = -wp[2]
        point = get_location_metres(global_frame, x, y, z)
        points.append(point)

    for point in points :
        h = point.alt-global_frame.alt
        if h < 2:
            h = 2
        cmds.add(Command( 0, 0, 0, 3, trajType, 0, 0, 0, 0, 0, 0, point.lat, point.lon, h))
    #add dummy last waypoint
    last_point = points[-1]
    cmds.add(Command( 0, 0, 0, 3, trajType, 0, 0, 0, 0, 0, 0, last_point.lat, last_point.lon, h))

    #print(" Upload new commands to vehicle")
    cmds.upload()

    # reset the command counter to 0 so the drone fly to the 1st waypoint and not to another one
    vehicle.commands.next = 0

    # download the mission
    # !!!!!!!!!!!!!!!!!!!!
    # NOT SURE IF IT IS USEFULL, MIGHT BE COMMENTED
    download_mission(vehicle)


if __name__ == '__main__':

    while True:

        if mode == 'GROUND':
            #--- Wait until a valid mission has been uploaded
            n_WP, missionList = get_current_mission(vehicle)
            time.sleep(2)


            arm_and_takeoff(10)


            if n_WP > 0:
                print ("A valid mission has been uploaded: takeoff!")
                mode = 'TAKEOFF'


        elif mode == 'TAKEOFF':

            #-- Add a fake waypoint at the end of the mission


            original_location = vehicle.location.global_relative_frame


            print(vehicle.location.global_relative_frame.lat)
            print(vehicle.location.global_relative_frame.lon)
            print(vehicle.location.global_relative_frame.alt)

            wp1 = get_location_metres(original_location,0,0,15)

            print(wp1)
            wp2 = get_location_metres(original_location,2,0,10)
            print(wp2)
            wp3 = get_location_metres(original_location,20,20,10)
            wp4 = get_location_metres(original_location,20,10,10)
            wp5 = get_location_metres(original_location,0,0,3)
            wp6 = get_location_metres(original_location,-5,-20,10)



            add_last_waypoint_to_mission(vehicle, wp1.lat, wp1.lon, wp1.alt)
            add_last_waypoint_to_mission(vehicle, wp2.lat, wp2.lon, wp2.alt)
            add_last_waypoint_to_mission(vehicle, wp3.lat, wp3.lon, wp3.alt)
            add_last_waypoint_to_mission(vehicle, wp4.lat, wp4.lon, wp1.alt)
            add_last_waypoint_to_mission(vehicle, wp5.lat, wp5.lon, wp1.alt)
            add_last_waypoint_to_mission(vehicle, wp6.lat, wp6.lon, wp1.alt)


            print("Home waypoint added to the mission")
            time.sleep(1)
            #-- Takeoff
            arm_and_takeoff(10)

            #-- Change the UAV mode to AUTO
            print("Changing to AUTO")
            ChangeMode(vehicle,"AUTO")

            #-- Change mode, set the ground speed
            vehicle.groundspeed = gnd_speed
            mode = 'MISSION'
            print ("Switch mode to MISSION")

        elif mode == 'MISSION':
            #-- Here we just monitor the mission status. Once the mission is completed we go back
            #-- vehicle.commands.cout is the total number of waypoints
            #-- vehicle.commands.next is the waypoint the vehicle is going to
            #-- once next == cout, we just go home

            print ("Current WP: %d of %d "%(vehicle.commands.next, vehicle.commands.count))
            if vehicle.commands.next == vehicle.commands.count:
                print ("Final waypoint reached: go back home")
                #-- First we clear the flight mission
                clear_mission(vehicle)
                print ("Mission deleted")

                #-- We go back home
                ChangeMode(vehicle,"RTL")
                mode = "BACK"

        elif mode == "BACK":
            if vehicle.location.global_relative_frame.alt < 1:
                print ("Switch to GROUND mode, waiting for new missions")
                mode = 'GROUND'
                ChangeMode(vehicle,"STABILIZE")


        time.sleep(0.5)

'''

def arm(vehicle):
    while not vehicle.is_armable:
        print("waiting to be armable")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed: time.sleep(1)

def takeoff_only(vehicle,altitude):

    print("Taking Off")
    vehicle.simple_takeoff(altitude)

    while True:
        v_alt = vehicle.location.global_relative_frame.alt
        print(">> Altitude = %.1f m"%v_alt)
        if v_alt >= altitude - 0.1:
            print("Target altitude reached")
            break
        time.sleep(1)

def clear_mission(vehicle):
    """
    Clear the current mission.
    """
    cmds = vehicle.commands
    vehicle.commands.clear()
    vehicle.commands.upload()


    # After clearing the mission you MUST re-download the mission from the vehicle
    # before vehicle.commands can be used again
    # (see https://github.com/dronekit/dronekit-python/issues/230)
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()


def add_last_waypoint_to_mission(                                       #--- Adds a last waypoint on the current mission file
        vehicle,            #--- vehicle object
        wp_Last_Latitude,   #--- [deg]  Target Latitude
        wp_Last_Longitude,  #--- [deg]  Target Longitude
        wp_Last_Altitude):  #--- [m]    Target Altitude
    """
    Upload the mission with the last WP as given and outputs the ID to be set
    """
    # Get the set of commands from the vehicle
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()


    # Save the vehicle commands to a list
    missionlist=[]

    for cmd in cmds:
        missionlist.append(cmd)



    # Modify the mission as needed. For example, here we change the


    #mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT

    wpLastObject = Command( 0, 0, 0, 3, mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT, 0, 0, 0, 0, 0, 0,
                           wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude)
    missionlist.append(wpLastObject)


    # Clear the current mission (command is sent when we call upload())
    cmds.clear()

    #Write the modified mission and flush to the vehicle
    for cmd in missionlist:
        cmds.add(cmd)
    cmds.upload()

    return (cmds.count)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint(vehicle):
    """
    Gets distance in metres to the current waypoint.
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint


'''