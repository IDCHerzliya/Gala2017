# 3DR_code
AUTHOR: Amy Chen (amyjchen@stanford.edu) + edits by Michelle Park
Code written by Amy Chen and Michelle Park (mpark97@stanford.edu)
Requires following the instructions in 
https://dev.3dr.com/starting-network.html and http://www.ddmckinnon.com/2015/12/30/idiots-guide-to-dronekit-python-a-journey-to-whoz-chillin/

# THINGS LEFT TO WORK ON:
- got the script to run on the drone but now it's being buggy (is taking too long to load packages, drone is getting disconnected. No changes have been made to the script since the last time it ran...) and it's the last day of CURIS. Steps to run:
(1) set up 3DR command line tool
(2) go to the desired folder and run "solo script pack" (with normal wifi)
(3) connect to the drone wifi and run "solo script run [script name]"
if you get an error about not being able to find a file while running it use rsync: "rsync -avz local/file/path/. root@10.1.1.10:/solo/path/." from https://dev.3dr.com/starting-installing.html. 
- MAVLINK allows for setting up geofences which would be much safer. 

# FOLDERS/extra files:
- archive: contains retired code from when we started learning how to program
  with the 3DR SDK
- drone_script: current drone scripts that are up-to-date. files described in 
  the next section.
- future: contains a folder titled "Safety." Unclear who wrote it. Suggests that
  this is how we can implement safety features from the get-go but neither me 
  nor Michelle remember creating it. 

# DRONE_SCRIPT:

- requirements.txt: file used for packing the script on the drone. Lists 
  packages the drone needs to import via the dual-wifi connection.

- dronekit: contains 3DR's SDK. DO NOT DELETE.

- keypoller.py & keypoller.pyc & keypollertest.py: keypoller allows for terminal 
 input at any point during the route. Run "python keypollertest.py" to 
 see it in action and to view a simple implementation. keypoller is also used 
 in several scripts.  

- engineering_missions: contains coordinates to each point in a route in CSV files. 
  Each file states the total number of points, latitute, longitute, and next possible  
  points for that path. Locations are destinations on campus. Each file (except mission_start.csv) assumes the drone is already at it proposed start location. AKA gates_packard.csv assumes you are already at gates, so the first coordinate is at the next coordinate from gates you should go to. 

- non_sim.py: This program takes in a string called "landmarks", in which each character represents a point to go to (g = gates, p = packard, h = hewlett). It will only run if a string starts and ends with g (must start and land at gates).

- test_takeoff.py: This function does a takeoff of the vehicle and landing. 
  USE THIS TO TEST CODE WHEN FIRST RUNNING AUTONOMOUS ROUTE ON DRONE. The program 
  commands the drone to reach an altitude of 5 meters and immediately land. Has not been tested on the drone yet. 

 
