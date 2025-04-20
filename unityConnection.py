import asyncio
import subprocess
import uuid
from pymavlink import mavutil

UDP_PORT = "14540"
DOCKER_IMAGE = "px4_docker"

def run_unique_px4_container():
    unique_id = uuid.uuid4().hex[:6]
    container_name = f"px4_{unique_id}"
    print(f"[INFO] Launching container: {container_name}")

    subprocess.run([
        "docker", "run", "-itd",
        "--name", container_name,
        "-p", f"{UDP_PORT}:{UDP_PORT}/udp",
        DOCKER_IMAGE
    ], check=True)

    print("[INFO] PX4 container starting...")
    return container_name

async def wait_for_heartbeat(master):
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat from sysid={master.target_system}, compid={master.target_component}")

async def arm_drone(master):
    print("Arming...")
    master.arducopter_arm()
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if msg and msg.result == 0:
        print("Arm ACK:", msg)
    else:
        raise RuntimeError("Failed to arm")

async def wait_for_altitude(master, min_alt=1000):  # mm
    print(f"Waiting to reach altitude > {min_alt / 1000:.1f} m...")
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg and msg.relative_alt >= min_alt:
            print(f"Reached altitude: {msg.relative_alt / 1000:.2f} m")
            break
        await asyncio.sleep(0.5)

async def set_takeoff_mode(master):
    print("Setting mode to TAKEOFF...")
    master.set_mode('TAKEOFF')
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack and ack.result == 0:
        print("Mode change ACK:", ack)
    else:
        raise RuntimeError("Failed to set TAKEOFF mode")

async def set_rtl_mode(master):
    print("Setting mode to RTL...")
    master.set_mode('RTL')
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack and ack.result == 0:
        print("Mode change ACK:", ack)
    else:
        raise RuntimeError("Failed to set RTL mode")

async def send_mission(master, lat, lon, alt):
    print("Sending mission...")
    master.waypoint_clear_all_send()

    takeoff = mavutil.mavlink.MAVLink_mission_item_message(
        master.target_system, master.target_component,
        seq=0,
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        current=0,
        autocontinue=1,
        param1=0, param2=0, param3=0, param4=0,
        x=0, y=0, z=alt
    )

    waypoint = mavutil.mavlink.MAVLink_mission_item_message(
        master.target_system, master.target_component,
        seq=1,
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        current=0,
        autocontinue=1,
        param1=0, param2=0, param3=0, param4=0,
        x=lat, y=lon, z=alt
    )

    master.mav.mission_count_send(master.target_system, master.target_component, 2)

    req = master.recv_match(type='MISSION_REQUEST', blocking=True, timeout=5)
    if req and req.seq == 0:
        master.mav.send(takeoff)
        print("[MISSION] Sent takeoff item.")

    req = master.recv_match(type='MISSION_REQUEST', blocking=True, timeout=5)
    if req and req.seq == 1:
        master.mav.send(waypoint)
        print("[MISSION] Sent waypoint item.")

    ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    if ack and ack.type == 0:
        print("[MISSION] Mission acknowledged.")

async def start_mission(master):
    print("Starting mission...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack and ack.result == 0:
        print("Mission start ACK:", ack)
    else:
        raise RuntimeError("Failed to start mission")

async def wait_until_waypoint_reached(master, target_seq):
    print(f"Waiting to reach waypoint #{target_seq}...")
    while True:
        msg = master.recv_match(type='MISSION_ITEM_REACHED', blocking=True, timeout=10)
        if msg and msg.seq == target_seq:
            print(f"[MISSION] Waypoint {msg.seq} reached.")
            break
        await asyncio.sleep(1)

async def main():
    container_name = run_unique_px4_container()

    try:
        master = mavutil.mavlink_connection(f'udp:10.0.0.1:{UDP_PORT}') # Replace with your actual IP
        await wait_for_heartbeat(master)
        await asyncio.sleep(5)  # Wait for the drone to be ready
        await arm_drone(master)
        await send_mission(master, lat=47.397751, lon=8.545607, alt=10) # Example coordinates
        await set_takeoff_mode(master)
        await wait_for_altitude(master, min_alt=2)
        await start_mission(master)
        await wait_until_waypoint_reached(master, target_seq=1)
        await set_rtl_mode(master)

    finally:
        print(f"[INFO] Cleaning up container: {container_name}")
        subprocess.run(["docker", "stop", container_name])
        subprocess.run(["docker", "rm", container_name])

if __name__ == '__main__':
    asyncio.run(main())
