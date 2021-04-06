#!/bin/bash

# Adapted by: Alejandra Martínez, Charalampos Efstratiou and Tom Antoine
# GNC subteam, group 1, GDP AVDC 2020-2021
# email:
# alejandra.martinez-farina@cranfield.ac.uk
# charalampos.efstratiou@cranfield.ac.uk
# tom.antoine@cranfield.ac.uk

# Based on a code by:
# author: Eric Johnson
# github: https://github.com/Intelligent-Quads

# Please feel free to use and modify this, but keep the above information. Thanks!

gnome-terminal \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I0" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I1" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I2" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I3" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I4" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I5" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I6" \
