'''
guided_route.py
WRITTEN BY AMY CHEN 
12 July 2016
----------------------------------------------------------------------
This program uses 3DR's DroneKit to fly a drone in a square formation
in the side yard of Gates Computer Science in GUIDED mode.
'''

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time, csv, math, argparse

# CONSTANTS
MAX_ALTITUDE = 5
MISSION_FILE_NAME = "waypoint_missions/mission.csv"
NUM_WAYPOINTS = 4

# Get the mission points
with open(MISSION_FILE_NAME) as csvfile:
	pointdict = csv.DictReader(csvfile)
	points = list(pointdict)

#OPTION PARSING FOR CONNECTION STRING
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
connection_string = args.connect
sitl = None

#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default(lat=float(points[0]['lat']), lon=float(points[0]['lon']))
    connection_string = sitl.connection_string()
    
#Connect to the Vehicle
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
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt 
        #Break and return from function just below target altitude.        
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
            print "Reached target altitude"
            break
        time.sleep(1)

def go_to_point(latitude, longitude, altitude): 
    print 'LAT: %s'%(latitude) + ', LONG: %s'%(longitude) + ', ALT: %s'%(altitude)
    point = LocationGlobalRelative(latitude, longitude, altitude)
    vehicle.simple_goto(point)
    # sleep so we can see the change in map
    time.sleep(10)

# takeoff
arm_and_takeoff(MAX_ALTITUDE)

print "Set default/target groundspeed to 0.5"
vehicle.groundspeed = 0.5

# go to points
point_num = 1
for point in points:
    print 'Going towards point #' + '%s'%(point_num)
    go_to_point(float(point['lat']), float(point['lon']), MAX_ALTITUDE)
    point_num = point_num + 1

# return home
print "Returning to Launch"
vehicle.mode = VehicleMode("RTL")

#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()