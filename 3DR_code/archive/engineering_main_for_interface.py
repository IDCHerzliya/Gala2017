'''
File: engineering_main.py
Authors: Amy Chen and Michelle Park
Date: 23 July 2016
----------------------------------------------------------
This program uses 3DR's DroneKit and keypoller (from a stackoverflow response) to fly multiple paths, depending on user input.

COMMANDS:
"p" (only during a mission): plays or pauses the program
"g", "h", "p", ... (between missions): picks a route. 
'''

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
from keypoller import *
import time, csv, math, sys, argparse  

#CONSTANTS
MAX_ALTITUDE = 10
TARGET_ALTITUDE = 5
'''
ROUTE COMBINATIONS
GH --> HG or HP
GP --> PG or PH
'''
MISSION_START = "./engineering_missions/mission_start.csv"
MISSION_GH = "./engineering_missions/gates_hewlett.csv"
MISSION_GP = "./engineering_missions/gates_packard.csv"
MISSION_HG = "./engineering_missions/hewlett_gates.csv"
MISSION_HP = "./engineering_missions/hewlett_packard.csv"
MISSION_PG = "./engineering_missions/packard_gates.csv"
MISSION_PH = "./engineering_missions/packard_hewlett.csv"

GROUNDSPEED = 1 #m/s
AIRSPEED = 2 #m/s

#INSTANCE VARIABLES
landmarks = "ghpg"
prevLandmark = "g"
above_max_alt = False
num_waypoints = 0
points = []
last = False

# ----- FUNCTION DEFINITIONS -----

def get_mission(mission):
    with open(mission) as csvfile:
        global points
        while len(points) > 0:
            points.pop()
        pointdict = csv.DictReader(csvfile)
        points = list(pointdict)

def get_next_missions(choice):
        if choice == 'h' and prevLandmark == 'g':
            print "going to Hewlett"
            return MISSION_GH
        if choice == 'p' and prevLandmark == 'g':
            print "going to Packard"
            return MISSION_GP
        if choice == 'g' and prevLandmark == 'h':
            print "going to Gates"
            return MISSION_HG
        if choice == 'p' and prevLandmark == 'h':
            print "going to Packard"
            return MISSION_HP
        if choice == 'g' and prevLandmark == 'p':
            print "going to Gates"
            return MISSION_PG
        if choice == 'h' and prevLandmark == 'p':
            print "going to Hewlett"
            return MISSION_PH
     
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

def set_speed(speed):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, #command
        0, #confirmation
        0, #param 1
        speed, # speed in metres/second
        0, 0, 0, 0, 0 #param 3 - 7
        )

    # Set airspeed using attribute, use this to control speed w/ airspeed instead of groundspeed
    # vehicle.airspeed = AIRSPEED #m/s

    # Set groundspeed using attribute
    vehicle.groundspeed = GROUNDSPEED #m/s
    
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def add_first_mission():

	print "Clear any existing commands"
	vehicle.commands.clear()

	print "Define/add new commands."
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the airself
	vehicle.commands.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, TARGET_ALTITUDE))

	global num_waypoints
	num_waypoints = int(points[0]['pts'])
        
	for point in points:
		vehicle.commands.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, float(point['lat']), float(point['lon']), 0))
        
	#dummy waypoint
	vehicle.commands.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, float(points[num_waypoints - 1]['lat']), float(points[num_waypoints - 1]['lon']), 0))

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
		vehicle.commands.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, float(point['lat']), float(point['lon']), 0))

	#dummy waypoint
	vehicle.commands.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, float(points[num_waypoints - 1]['lat']), float(points[num_waypoints - 1]['lon']), 0))
        
	if last == True :
		num_waypoints = num_waypoints - 2
    
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

def start_mission():
    # From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
    if above_max_alt != True:
        print "Starting mission."
        # Set mode to AUTO to start mission
        vehicle.mode = VehicleMode("AUTO")
        set_speed(0.25)

def monitor_mission():
    # Monitor mission. 
    # Demonstrates getting and setting the command number 
    # Uses distance_to_current_waypoint(), a convenience function for finding the 
    #   distance to the next waypoint.
    nextwaypoint = vehicle.commands.next
    while vehicle.location.global_relative_frame.alt > 0:
        nextwaypoint = vehicle.commands.next
        print 'Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint())
        print vehicle.airspeed
        if nextwaypoint == num_waypoints + 1: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
			print "Exit 'standard' mission when start heading to final waypoint"
			vehicle.commands.next = 0
			vehicle.commands.clear()
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

#'''OPEN SOCKET'''
# socket replaces landmarks with actual landmark file

# Listens for altitude changes, calls max_altitude_check()
vehicle.add_attribute_listener('location.global_relative_frame', max_altitude_check)
# Wait 2s so callback can be notified before the observer is removed
time.sleep(2)
# Remove observer - specifying the attribute and previously registered callback function
# vehicle.remove_message_listener('location.global_relative_frame.alt', max_altitude_check)

add_first_mission()
arm_and_takeoff(TARGET_ALTITUDE)
start_mission()
monitor_mission()
while not (landmarks.startswith("g") and landmarks.endswith("g")):
    # TODO SOCKET INPUT CHECK FOR G AND G
    landmarks = raw_input('Type a new route: ')
i = 1
while i < len(landmarks) :
    if i == (len(landmarks) - 1):
        last = True
    get_mission(get_next_missions(landmarks[i]))
    add_mission()
    arm()
    start_mission()
    if above_max_alt != True:
        monitor_mission() 
        vehicle.mode = VehicleMode("GUIDED")
    prevLandmark = landmarks[i]
    i = i + 1
        
'''LAND DRONE'''    
print 'Land'
vehicle.mode = VehicleMode("LAND")
while vehicle.location.global_relative_frame.alt > 0:
    print " Altitude: ", vehicle.location.global_relative_frame.alt
    time.sleep(1)
end_mission()

'''CLOSE SOCKET'''