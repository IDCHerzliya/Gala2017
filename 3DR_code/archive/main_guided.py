'''
File: main.py
Authors: Amy Chen and Michelle Park
Date: 20 July 2016
----------------------------------------------------------
UNFINISHED
This program uses 3DR's DroneKit and keypoller (from a stackoverflow response) to fly multiple paths, depending on user input.

COMMANDS:
"p" (only during a mission): plays or pauses the program
"a", "b", "c", ... (between missions): picks a route. 
'''

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
from keypoller import *
import time, csv, math, sys, argparse  

#CONSTANTS
MAX_ALTITUDE = 10
TARGET_ALTITUDE = 5
GROUNDSPEED = 0.5 # m/s

'''
A GATES --> THE OVAL
B GATES --> THE QUAD
C THE OVAL --> ENGINEERING QUAD
D THE OVAL --> ARTS DISTRICT
E THE QUAD --> ENGINEERING QUAD
F THE QUAD --> ARTS DISTRICT
G ENGINEERING QUAD --> GATES
H ARTS DISTRICT --> GATES

ROUTE COMBINATIONS
A --> CG or DH
B --> EG or FH
'''
MISSION_START = "missions/mission_start.csv"
MISSION_A = "missions/mission_A.csv"
MISSION_B = "missions/mission_B.csv"
MISSION_C = "missions/mission_C.csv"
MISSION_D = "missions/mission_D.csv"
MISSION_E = "missions/mission_E.csv"
MISSION_F = "missions/mission_F.csv"
MISSION_G = "missions/mission_G.csv"
MISSION_H = "missions/mission_H.csv"

#INSTANCE VARIABLES
above_max_alt = False
num_waypoints = 0
points = []
next_points = []

# ----- FUNCTION DEFINITIONS -----

def get_mission(mission):
    with open(mission) as csvfile:
        global points
        while len(points) > 0:
            points.pop()
        pointdict = csv.DictReader(csvfile)
        points = list(pointdict)
        print points
        get_next_missions()

def get_next_missions():
    global next_points
    while len(next_points) > 0:
        next_points.pop()
    if points[0]["A"] == "next":
        next_points.append(MISSION_A)
    if points[0]["B"] == "next":
        next_points.append(MISSION_B)
    if points[0]["C"] == "next":
        next_points.append(MISSION_C)
    if points[0]["D"] == "next":
        next_points.append(MISSION_D)
    if points[0]["E"] == "next":
        next_points.append(MISSION_E)
    if points[0]["F"] == "next":
        next_points.append(MISSION_F)
    if points[0]["G"] == "next":
        next_points.append(MISSION_G)
    if points[0]["H"] == "next":
        next_points.append(MISSION_H)
        
def print_options():
    if len(next_points) == 1:
        print "ONLY ONE OPTION. Semi-automate later."
    for point in next_points:
        if point == MISSION_A:
            print "type \"a\" to go to the oval"
        if point == MISSION_B:
            print "type \"b\" to go to the quad"
        if point == MISSION_C:
            print "type \"c\" to go to the engineering quad"
        if point == MISSION_D:
            print "type \"d\" to go to the arts district"
        if point == MISSION_E:
            print "type \"e\" to go to the engineering quad"
        if point == MISSION_F:
            print "type \"f\" to go to the arts district"
        if point == MISSION_G:
            print "type \"g\" to go back to gates"
        if point == MISSION_H:
            print "type \"h\" to go back to gates"
    while True:
        choice = keyPoller.poll()
        if choice != None:
        	print choice
        if choice == 'a' and MISSION_A in next_points:
            print "going to the oval"
            return MISSION_A
        if choice == 'b' and MISSION_B in next_points:
            print "going to the quad"
            return MISSION_B
        if choice == 'c' and MISSION_C in next_points:
            print "going to the engineering quad"
            return MISSION_C
        if choice == 'd' and MISSION_D in next_points:
            print "going to the arts district"
            return MISSION_D
        if choice == 'e' and MISSION_E in next_points:
            print "going to the engineering quad"
            return MISSION_E
        if choice == 'f' and MISSION_F in next_points:
            print "going to the arts district"
            return MISSION_F
        if choice == 'g' and MISSION_G in next_points:
            print "going to gates"
            return MISSION_G
        if choice == 'h' and MISSION_H in next_points:
            print "going to gates"
            return MISSION_H
     
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
    if nextwaypoint == 0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

def add_first_mission():
    # gets the drone to the initial position
	print "Clear any existing commands"
	vehicle.commands.clear()

	print "Define/add new commands."
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the airself
	vehicle.commands.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, TARGET_ALTITUDE))

	#dummy waypoint
	vehicle.commands.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, float(points[0]['lat']), float(points[0]['lon']), 0))

	print "Upload new commands to vehicle"
	vehicle.commands.upload()  
	vehicle.commands.wait_ready()

def add_mission():

	print "Clear any existing commands"
	vehicle.commands.clear()

	print "Define/add new commands."
	global num_waypoints
	num_waypoints = int(points[0]['pts'])
    
	for point in points:
        #attempting to change speed
		vehicle.commands.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, float(point['lat']), float(point['lon']), 0))

	#dummy waypoint
	vehicle.commands.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, float(points[num_waypoints - 1]['lat']), float(points[num_waypoints - 1]['lon']), 0))
    
	print "Upload new commands to vehicle"
	vehicle.commands.upload() 
	vehicle.commands.wait_ready()

def arm():
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
    
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    arm()
    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while vehicle.location.global_relative_frame.alt >= 0 :
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print "Reached target altitude"
            break
        time.sleep(1)
    vehicle.mode = VehicleMode("AUTO")

def start_mission():
    # From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
    if above_max_alt != True:
        print "Starting mission. Type \"p\" to play/pause."

def go_to_point(latitude, longitude, altitude): 
    print 'LAT: %s'%(latitude) + ', LONG: %s'%(longitude) + ', ALT: %s'%(altitude)
    point = LocationGlobalRelative(latitude, longitude, altitude)
    vehicle.simple_goto(point, GROUNDSPEED)
    # sleep so we can see the change in map
    time.sleep(10)
    
def monitor_mission():
    for point in points:
        go_to_point(float(point['lat']), float(point['lon']), TARGET_ALTITUDE)
        print vehicle.groundspeed
        print vehicle.location.global_relative_frame
'''
def monitor_mission():
    # Monitor mission. 
    # Demonstrates getting and setting the command number 
    # Uses distance_to_current_waypoint(), a convenience function for finding the 
    #   distance to the next waypoint.
    nextwaypoint = vehicle.commands.next
    while vehicle.location.global_relative_frame.alt > 0:
        if keyPoller.poll() == 'p':
            if vehicle.mode == VehicleMode("GUIDED"):
                vehicle.mode = VehicleMode("LOITER")
                print "STOP"
            else:
                vehicle.mode = VehicleMode("GUIDED")
                print "GO"
        nextwaypoint = vehicle.commands.next
        print 'Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint())
        print vehicle.groundspeed
        if nextwaypoint == num_waypoints + 1: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
			print "Exit 'standard' mission when start heading to final waypoint"
			vehicle.commands.next = 0
			vehicle.commands.clear()
			break;
        time.sleep(1)
'''

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
    #vehicle.mode = VehicleMode("GUIDED")
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
#GET THE MISSION START

get_mission(MISSION_START)
    
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

'''OPEN SOCKET'''

# Listens for altitude changes, calls max_altitude_check()
vehicle.add_attribute_listener('location.global_relative_frame', max_altitude_check)
# Wait 2s so callback can be notified before the observer is removed
time.sleep(2)

vehicle.groundspeed = GROUNDSPEED

with KeyPoller() as keyPoller:
    add_first_mission()
    arm_and_takeoff(TARGET_ALTITUDE)
    start_mission()
    monitor_mission() 
    while True:
        # if there are no more points, you have reached the end and are done.
        if len(next_points) == 0:
            break
        mission = print_options()
        get_mission(mission)
        arm()
        start_mission()
        if above_max_alt != True:
			monitor_mission() 
			vehicle.mode = VehicleMode("GUIDED")
        
    '''LAND DRONE'''    
    print 'Land'
    vehicle.mode = VehicleMode("LAND")
    while vehicle.location.global_relative_frame.alt > 0:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        time.sleep(1)
    end_mission()

'''CLOSE SOCKET'''