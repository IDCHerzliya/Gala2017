import math
from dronekit import *
from pymavlink.mavutil import mavlink
import time
import threading
import sys

class Safety():
    def _init_(self, vehicle, update_rate = 15):
        self.vehicle = vehicle
        self.fence_breach = False
        self.update_rate = update_rate
    
    def brake(self):
        mode = self.vehicle.mode
        msg  = self.vehicle.message_factory.set_mode_encode(0,
                                                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                17)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        self.vehicle.mode = mode
    
    def enable_fence(self, alt_floor, alt_ceiling ,radius):
        self.fence_enabled = True
        self.fence_floor = alt_floor
        self.fence_ceiling = alt_ceiling
        self.fence_radius = radius
        fence_thread = threading.Thread(target = self._enforce_fence)
        fence_thread.start()

    #disble_fence - Disble the geo fence
    def disble_fence(self):
        self.fence_enabled = False
        self.fence_breach = False
        
    
    def _enforce_fence(self):
        while self.fence_enabled and self.vehicle.mode == "GUIDED":
            try:
                veh_loc = self.vehicle.location.global_relative_frame
                home_loc = self.vehicle.home_location
                if home_loc is None:
                    cmds = self.vehicle.commands
                    cmds.download()
                    cmds.wait_ready()
                    home_loc = self.vehicle.home_location

                latlon_to_m = 111319.5   # converts lat/lon to meters
                dist_xy_home = math.sqrt(((home_loc.lat - veh_loc.lat) * latlon_to_m)**2 +((home_loc.lon - veh_loc.lon) * latlon_to_m)**2)
                if dist_xy_home >= self.fence_radius or veh_loc.alt < self.fence_floor or veh_loc.alt > self.fence_ceiling:
                    self.brake()
                    self.fence_breach = True
                    print '\033[91m' + "Fence Breach! Braking" + '\033[0m'
                    time.sleep(1)
                    break
            except:
                pass

            time.sleep(0.1)