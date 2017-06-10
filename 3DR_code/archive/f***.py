'''
f***.py
WRITTEN BY AMY CHEN 
14 July 2016
----------------------------------------------------------------------
This program has a drone take off and land in Guided mode, to be used for safe testing of code. 
This code also contains the practice implementation of keypoller, which is used to play and pause the drone mid-route
'''

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time, csv, math, argparse, sys, os
from keypoller import *

# CONSTANTS
MAX_ALTITUDE = 20

#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
connection_string = args.connect
sitl = None

#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()
# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    """ Arms vehicle and fly to aTargetAltitude."""

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

        
    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True    

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    with KeyPoller() as keyPoller:
        while True:
            print " Altitude: ", vehicle.location.global_relative_frame.alt 
            c = keyPoller.poll()
            print "poll: ", c
            #Break and return from function just below target altitude.        
            if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
                print "Reached target altitude"
                break
            time.sleep(1)

# takeoff
arm_and_takeoff(MAX_ALTITUDE)

# return home
print "Returning to Launch"
vehicle.mode = VehicleMode("LAND")

while vehicle.location.global_relative_frame.alt > 0:
    print " Altitude: ", vehicle.location.global_relative_frame.alt
    time.sleep(1)

#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()