"""
By: Ilan Mulkandov
Assignment: Robust Single Drone Mission
"""

import asyncio
import subprocess
import uuid
import math
from pymavlink import mavutil

# --- Constants ---
# IP Configuration
LOCAL_IP = "10.0.0.8"
DOCKER_IMAGE = "px4_sitl"

# PX4 Home Location
PX4_HOME_LAT = 47.397751
PX4_HOME_LON = 8.545607
PX4_HOME_ALT = 488.13123

# Ports
CONTROL_PORT = 14540
QGC_PORT = 14550

# Flight Parameters
TAKEOFF_ALTITUDE = 10.0
REL_ALTITUDE_MISSION = 10.0
ACCEPTANCE_RADIUS = 5.0
HOLD_TIME_SECONDS = 2
BOOT_WAIT_TIME = 10
GPS_POLL_INTERVAL = 1
HEARTBEAT_TIMEOUT = 1

# Watchdog Constants (Added Missing Definitions)
STUCK_THRESHOLD_METERS = 0.2
WATCHDOG_TIMEOUT_LOOPS = 25  # Approx 5 seconds
LOOP_SLEEP_TIME = 0.2

# MAVLink Mode Flags & Commands
OFFBOARD_MODE_FLAG = 6
AUTO_MODE_FLAG = 4
LOITER_MODE_FLAG = 3
MISSION_MODE_FLAG = 4
ARM_FORCE_FLAG = 1
VELOCITY_MASK = 0b110111000111

# Geometry Constants
EARTH_RADIUS = 6378137.0
OFFSET_FORWARD = 50
OFFSET_SIDE = 30
OFFSET_RETURN = 0

# --- Helper Functions ---

def addOffsetToCoords(lat, lon, offsetX, offsetY):
    """
    Adds meters to coordinates (X=North, Y=East).
    Calculates new latitude and longitude based on Earth's radius.
    """
    dLat = offsetX / EARTH_RADIUS
    dLon = offsetY / (EARTH_RADIUS * math.cos(math.pi * lat / 180))
    return lat + (dLat * 180 / math.pi), lon + (dLon * 180 / math.pi)


def getDistanceMeters(lat1, lon1, lat2, lon2):
    """
    Calculates the distance in meters between two GPS coordinates using the Haversine formula.
    """
    radiusCalc = 6371000 
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    
    aVal = math.sin(dLat / 2) * math.sin(dLat / 2) + \
            math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * \
            math.sin(dLon / 2) * math.sin(dLon / 2)
            
    cVal = 2 * math.atan2(math.sqrt(aVal), math.sqrt(1 - aVal))
    return radiusCalc * cVal


def runPx4Container():
    """
    Launches a Docker container for the PX4 simulation.
    """
    containerName = f"px4_sitl_{str(uuid.uuid4())[:4]}"
    print(f"[INIT] Launching container: {containerName}")
    
    cmd = [
        "docker", "run", "-itd", "--rm",
        "--name", containerName,
        "-e", f"QGC_IP={LOCAL_IP}",
        "-e", f"CONTROL_IP={LOCAL_IP}",
        "-e", f"PX4_HOME_LAT={PX4_HOME_LAT}",
        "-e", f"PX4_HOME_LON={PX4_HOME_LON}",
        "-e", f"PX4_HOME_ALT={PX4_HOME_ALT}",
        "-e", f"QGC_PORT={QGC_PORT}",
        "-e", f"CONTROL_PORT={CONTROL_PORT}", 
        "-p", f"{CONTROL_PORT}:{CONTROL_PORT}/udp",
        "-p", f"{QGC_PORT}:{QGC_PORT}/udp", 
        DOCKER_IMAGE
    ]
    
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
    return containerName

# --- Drone Logic Class ---

class DroneController:
    """
    Manages the connection and control logic for the drone.
    """
    def __init__(self, port):
        self.port = port
        self.master = None
        self.homeLat = 0
        self.homeLon = 0

    async def connectAndWait(self):
        """
        Establishes MAVLink connection and waits for the first heartbeat.
        """
        connString = f'udp:{LOCAL_IP}:{self.port}'
        print(f"[Drone] Connecting via UDP on {connString}...")
        self.master = mavutil.mavlink_connection(connString)
        
        print(f"[Drone] Waiting for Heartbeat...")
        while True:
            try:
                self.master.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT)
                print(f"[Drone] Connected! System ID: {self.master.target_system}")
                break
            except Exception:
                await asyncio.sleep(1)
                
        print(f"[Drone] Requesting Data Streams...")
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4,
            1
        )

    async def waitForHome(self):
        """
        Polls the drone until a valid Home Position is received.
        """
        print(f"[Drone] Waiting for GPS/Home lock...")
        while True:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 
                0, 0, 0, 0, 0, 0, 0, 0
            )
            msg = self.master.recv_match(type='HOME_POSITION', blocking=True, timeout=1)
            
            if msg:
                self.homeLat = msg.latitude / 1e7
                self.homeLon = msg.longitude / 1e7
                print(f"[Drone] Home Locked: {self.homeLat}, {self.homeLon}")
                break
            await asyncio.sleep(GPS_POLL_INTERVAL)

    async def offboardTakeoff(self, targetAlt):
        """
        Performs an offboard takeoff by sending velocity setpoints.
        """
        print(f"[Drone] Taking off to {targetAlt}m (Offboard)...")
        
        startAlt = 0
        for _ in range(10):
            mMsg = self.master.recv_match(type='VFR_HUD', blocking=True, timeout=1)
            if mMsg: 
                startAlt = mMsg.alt
                break
        
        targetAbs = startAlt + targetAlt

        while True:
            # Send velocity command (UP)
            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                VELOCITY_MASK, 
                0, 0, 0, 
                0, 0, -3.0, 
                0, 0, 0, 
                0, 0
            )
            
            # Ensure drone is Armed
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, ARM_FORCE_FLAG, 0, 0, 0, 0, 0, 0
            )
            
            # Ensure Offboard mode is active
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                OFFBOARD_MODE_FLAG,
                0, 0, 0, 0, 0
            )

            mMsg = self.master.recv_match(type='VFR_HUD', blocking=False)
            if mMsg and mMsg.alt >= targetAbs:
                print(f"[Drone] Airborne at {mMsg.alt:.1f}m!")
                break
            await asyncio.sleep(0.1)

    async def switchToHold(self):
        """
        Switches the drone to HOLD (Loiter) mode.
        """
        print(f"[Drone] Switching to HOLD (Loiter)...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            AUTO_MODE_FLAG,
            LOITER_MODE_FLAG,
            0, 0, 0, 0
        )
        await asyncio.sleep(HOLD_TIME_SECONDS)

    async def uploadTriangleMission(self, altRel):
        """
        Uploads a triangle shaped mission.
        """
        print(f"[Drone] Uploading Mission...")
        self.master.waypoint_clear_all_send()
        self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=2)

        # Point 1: Forward
        wp1Lat, wp1Lon = addOffsetToCoords(self.homeLat, self.homeLon, OFFSET_FORWARD, 0)
        # Point 2: Side
        wp2Lat, wp2Lon = addOffsetToCoords(self.homeLat, self.homeLon, OFFSET_SIDE, OFFSET_SIDE)
        # Point 3: Return
        wp3Lat, wp3Lon = addOffsetToCoords(self.homeLat, self.homeLon, 0, OFFSET_RETURN)

        points = [
            (self.homeLat, self.homeLon, altRel),  # 0: Dummy Start
            (wp1Lat, wp1Lon, altRel),              # 1
            (wp2Lat, wp2Lon, altRel),              # 2
            (wp3Lat, wp3Lon, altRel)               # 3
        ]

        self.master.mav.mission_count_send(
            self.master.target_system,
            self.master.target_component,
            len(points)
        )

        for i in range(len(points)):
            msg = self.master.recv_match(
                type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'],
                blocking=True,
                timeout=2
            )
            if msg:
                pLat, pLon, pAlt = points[msg.seq]
                
                # Param 2 = Acceptance Radius (5m)
                wp = mavutil.mavlink.MAVLink_mission_item_int_message(
                    self.master.target_system,
                    self.master.target_component,
                    msg.seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 1, 
                    0, ACCEPTANCE_RADIUS, 0, 0, 
                    int(pLat * 1e7),
                    int(pLon * 1e7),
                    int(pAlt)
                )
                self.master.mav.send(wp)
                print(f"[Drone] Sent WP {msg.seq}")
        
        self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=2)
        print(f"[Drone] Mission Uploaded.")
        return points

    async def startMissionAuto(self):
        """
        Sets the current mission item to 1 and switches to AUTO mode.
        """
        print(f"[Drone] Switching to AUTO MISSION...")
        # Skip waypoint 0
        self.master.mav.mission_set_current_send(
            self.master.target_system,
            self.master.target_component,
            1
        )
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            AUTO_MODE_FLAG,
            MISSION_MODE_FLAG,
            0, 0, 0, 0
        )
        await asyncio.sleep(0.5)

    async def monitorMissionProgress(self, points):
        """
        Monitors the drone's position with a Watchdog to prevent stuck state.
        """
        print(f"[Drone] Monitoring Mission with Watchdog...")
        currentWpIndex = 1
        totalWps = len(points)
        
        # Watchdog variables
        stuckCounter = 0
        lastDist = 99999.0

        while True:
            # 1. Update current index from drone report
            msgCurr = self.master.recv_match(type='MISSION_CURRENT', blocking=False)
            if msgCurr:
                if msgCurr.seq > currentWpIndex:
                    print(f"[Drone] Completed WP #{currentWpIndex}. Moving to #{msgCurr.seq}...")
                    currentWpIndex = msgCurr.seq
                    stuckCounter = 0 # Reset watchdog on progress

            # 2. Check for completion
            if currentWpIndex >= totalWps - 1:
                tLat, tLon, _ = points[-1]
                mMsg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                if mMsg:
                    cLat, cLon = mMsg.lat / 1e7, mMsg.lon / 1e7
                    dist = getDistanceMeters(cLat, cLon, tLat, tLon)
                    
                    print(f"[Drone] Final Approach: {dist:.1f}m", end='\r')
                    
                    if dist < ACCEPTANCE_RADIUS:
                        print(f"\n[Drone] Mission Loop Finished.")
                        break
            else:
                mMsg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                if mMsg:
                    tLat, tLon, _ = points[currentWpIndex]
                    cLat, cLon = mMsg.lat / 1e7, mMsg.lon / 1e7
                    dist = getDistanceMeters(cLat, cLon, tLat, tLon)
                    print(f"[Drone] Flying to WP {currentWpIndex}: {dist:.1f}m", end='\r')
                    
                    # --- Watchdog Logic ---
                    if abs(lastDist - dist) < STUCK_THRESHOLD_METERS:
                        stuckCounter += 1
                    else:
                        stuckCounter = 0
                    
                    lastDist = dist
                    
                    if stuckCounter > WATCHDOG_TIMEOUT_LOOPS:
                        print(f"\n[Drone] Watchdog Triggered! Resending WP {currentWpIndex} command...")
                        # Resend Auto Mode
                        self.master.mav.command_long_send(
                            self.master.target_system,
                            self.master.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                            0,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            AUTO_MODE_FLAG,
                            MISSION_MODE_FLAG,
                            0, 0, 0, 0
                        )
                        # Resend Current Waypoint
                        self.master.mav.mission_set_current_send(
                            self.master.target_system,
                            self.master.target_component,
                            currentWpIndex
                        )
                        stuckCounter = 0 # Reset

            await asyncio.sleep(LOOP_SLEEP_TIME)

    async def landNow(self):
        """
        Executes a LAND command and waits for disarm.
        """
        print(f"\n[Drone] Landing...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
        while True:
            mMsg = self.master.recv_match(type='VFR_HUD', blocking=True, timeout=1)
            if mMsg:
                # Filter negative altitude noise for display
                altDisplay = mMsg.alt if mMsg.alt > 0.1 else 0.0
                print(f"[Drone] Landing Alt: {altDisplay:.2f}m", end='\r')
                
                hbMsg = self.master.recv_match(type='HEARTBEAT', blocking=False)
                if hbMsg and not (hbMsg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    print(f"\n[Drone] Disarmed & Landed.")
                    break
            await asyncio.sleep(0.5)

# --- Main Execution ---

async def main():
    print("--- STARTING SWARM MISSION (SINGLE DRONE) ---")
    
    containerName = runPx4Container()
    
    try:
        print(f"Waiting {BOOT_WAIT_TIME}s for container boot...")
        await asyncio.sleep(BOOT_WAIT_TIME)
        
        drone = DroneController(port=CONTROL_PORT)
        
        await drone.connectAndWait()
        
        await drone.waitForHome()
        
        # Phase 1: Takeoff
        await drone.offboardTakeoff(targetAlt=TAKEOFF_ALTITUDE)
        
        # Phase 2: Stabilize
        await drone.switchToHold()
        
        # Phase 3: Upload Mission
        points = await drone.uploadTriangleMission(altRel=REL_ALTITUDE_MISSION)
        
        # Phase 4: Execute & Monitor
        await drone.startMissionAuto()
        await drone.monitorMissionProgress(points)
        
        # Phase 5: Land
        await drone.landNow()
        
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
    finally:
        print(f"[INFO] Cleaning up container {containerName}")
        subprocess.run(["docker", "stop", containerName])

if __name__ == '__main__':
    asyncio.run(main())