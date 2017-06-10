'''
File: route_waypoints.py
Author: Michelle Park
Date: 7 July 2016
----------------------------------------------------------------------
This program uses 3DR's DroneKit to fly a drone in a square formation
in the side yard of Gates Computer Science using waypoints in AUTO mode. 
The waypoints are positioned to form a square of a specified side length 
around the specified LocationGlobal (aLocation).
NEXT STEPS:
 -set commands to Gates side-yard coordinates
 -figure out play/pause options
'''
#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
import sys
from pymavlink import mavutil

#Constants
MAX_ALTITUDE = 2
TARGET_ALTITUDE = 10
SQUARE_SIDE_LENGTH = 50
MISSION_FILE_NAME = "waypoint_missions/gates_square.txt"

#Instance variables
above_max_alt = False

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

# ----- FUNCTION DEFINITIONS -----
def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

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

def distance_to_current_waypoint():
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

def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.

def add_square_mission(aLocation, aSize):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).
    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """ 
    cmds = vehicle.commands
    print " Clear any existing commands"
    cmds.clear() 
    print " Define/add new commands."
    #Add new commands. The meaning/order of the parameters is documented in the Command class. 
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = get_location_metres(aLocation, aSize, -aSize)
    point2 = get_location_metres(aLocation, aSize, aSize)
    point3 = get_location_metres(aLocation, -aSize, aSize)
    point4 = get_location_metres(aLocation, -aSize, -aSize)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 11))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 12))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 13))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))
    #add dummy waypoint "5" at point 4 (lets us know when have reached destination)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))    
    print " Upload new commands to vehicle"
    cmds.upload()

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)
    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)
    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print "Reached target altitude"
            break;
        time.sleep(1)

def start_mission():
    # From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
    arm_and_takeoff(TARGET_ALTITUDE)
    print "Starting mission"
    # Reset mission set to first (0) waypoint
    vehicle.commands.next=0
    # Set mode to AUTO to start mission
    vehicle.mode = VehicleMode("AUTO")

# Monitor mission. 
# Demonstrates getting and setting the command number 
# Uses distance_to_current_waypoint(), a convenience function for finding the distance to the next waypoint.
def monitor_mission():
    while True:
        nextwaypoint=vehicle.commands.next
        print 'Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint())
        if nextwaypoint==3: #Skip to next waypoint
            print 'Skipping to Waypoint 5 when reach waypoint 3'
            vehicle.commands.next = 5
        if nextwaypoint==5: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
            print "Exit 'standard' mission when start heading to final waypoint (5)"
            break;
        time.sleep(1)

def end_mission():
    # Removes safety check listener
    vehicle.remove_message_listener('location.global_relative_frame.alt', max_altitude_check)
    # Close vehicle object before exiting script
    print "Close vehicle object"
    vehicle.close()
    # Shut down simulator if it was started.
    if sitl is not None:
        sitl.stop()

#https://discuss.dronekit.io/t/best-way-to-build-app-using-dronekit-python-and-differences-using-mavlink/294
def land_at_curr_location():
    vehicle.mode = VehicleMode("GUIDED")
    print("Landing...")
    vehicle.mode = VehicleMode('LAND')

#Callback to print the location in global frames. 'value' is the updated value
def max_altitude_check(self, attr_name, value):
    global above_max_alt
    if vehicle.location.global_relative_frame.alt > MAX_ALTITUDE and not above_max_alt:
        print "Altitude is past set limit. ", value
        land_at_curr_location()
        above_max_alt = True

# ----- FUNCTION CALLS -----
# Listens for altitude changes, calls max_altitude_check()
vehicle.add_attribute_listener('location.global_relative_frame', max_altitude_check)
# Wait 2s so callback can be notified before the observer is removed
time.sleep(2)
# Remove observer - specifying the attribute and previously registered callback function
print 'Create a new mission (for current location)'
add_square_mission(vehicle.location.global_frame, SQUARE_SIDE_LENGTH)
start_mission()
monitor_mission()
print 'Return to launch'
vehicle.mode = VehicleMode("RTL")
end_mission()