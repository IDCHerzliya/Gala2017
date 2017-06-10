'''
File: route_waypoints.py
Author: Michelle Park
7 July 2016
----------------------------------------------------------------------
This program uses 3DR's DroneKit to fly a drone in a square formation
in the side yard of Gates Computer Science using waypoints in AUTO mode. 
'''

#CONSTANTS
TARGET_ALTITUDE = 4 # in meters
MISSION_FILE_NAME = "waypoint_missions/gates_square.txt"

from dronekit import *
import time

#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
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

def readmission(aFileName):
    """
    Load a mission from a file into a list.

    This function is used by upload_mission().
    """
    print "Reading mission from file: %s\n" % aFileName
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            elif line.startswith(' '):
                    return missionlist;
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, 
                    ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist

# uses readmission()
def upload_mission(aFileName):
        """
        Upload a mission from a file.
        """
        #Read mission from file
        missionlist = readmission(aFileName)
        print "\nUpload mission from a file: %s" % aFileName
        #Clear existing mission from vehicle
        print ' Clear mission'
        cmds = vehicle.commands
        cmds.clear()
        #Add new mission to vehicle
        for command in missionlist:
            cmds.add(command)
        print ' Upload mission'
        vehicle.commands.upload()

def end_mission():
    #Close vehicle object before exiting script
    print "Close vehicle object"
    vehicle.close()
    # Shut down simulator if it was started.
    if sitl is not None:
        sitl.stop()

upload_mission(MISSION_FILE_NAME);
# Set the vehicle into auto mode to start mission
vehicle.mode = VehicleMode("AUTO")

if missionlist[]





