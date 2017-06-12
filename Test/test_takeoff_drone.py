'''
test_takeoff_drone.py
WRITTEN BY AMY CHEN 13 July 2016
MODIFIED BY JESSICA CAUCHARD 12 June 2017
----------------------------------------------------------------------
This program has a drone take off and land in Guided mode, to be used for safe testing of code. 
'''

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time, csv, math, argparse

# CONSTANTS
MAX_ALTITUDE = 3

target = 'udpin:0.0.0.0:14550'
print 'Connecting to ' + target + '...'
vehicle = connect(target, wait_ready=True)
print vehicle

def arm_and_takeoff(aTargetAltitude):
    """ Arms vehicle and fly to aTargetAltitude."""

    #global vehicle

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
    #print("Connecting to vehicle on: %s" % (connection_string,))
    #vehicle = connect(connection_string, wait_ready=True)
    
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt 
        #Break and return from function just below target altitude.        
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
            print "Reached target altitude"
            # Print location information for `vehicle` in all frames (default printer)
            print "Global Location: %s" % vehicle.location.global_frame
            print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
            print "Local Location: %s" % vehicle.location.local_frame    #NED

            # Print altitudes in the different frames (see class definitions for other available information)
            print "Altitude (global frame): %s" % vehicle.location.global_frame.alt
            print "Altitude (global relative frame): %s" % vehicle.location.global_relative_frame.alt
            print "Altitude (NED frame): %s" % vehicle.location.local_frame.down
            break
        time.sleep(1)

# takeoff
arm_and_takeoff(MAX_ALTITUDE)

#vehicle.simple_goto(LocationGlobalRelative(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, -10))

# return home
print "Returning to Launch"
vehicle.mode = VehicleMode("LAND")

while vehicle.location.global_relative_frame.alt > 0:
    print " Altitude: ", vehicle.location.global_relative_frame.alt
    time.sleep(1)

#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()