import asyncio
import subprocess
import uuid
import json
from pymavlink import mavutil

# Global configuration
global control_port,qgc_port
control_port = 14540 # can be change but not recommended if you use it only until 10 containers
qgc_port = 14550 # can be change but not recommended if you use it only until 10 containers
DOCKER_IMAGE = "px4_sitl" # Replace with your actual Docker image name
LOCAL_IP = "10.0.0.16"  # Replace with your actual local IP
PX4_HOME_LAT=47.397751 # Replace with your actual latitude
PX4_HOME_LON=8.545607 # Replace with your actual longitude
PX4_HOME_ALT=488.13123 # Replace with your actual altitude

def run_unique_px4_container():
    """
    Launch a PX4 SITL container with unique name and available ports.
    Returns the container name.
    """
    global control_port, qgc_port
    unique_id = uuid.uuid4().hex[:6]
    container_name = f"{DOCKER_IMAGE}_{unique_id}"
    print(f"[INFO] Launching container: {container_name}")

    qgc_port = get_next_available_port(qgc_port, qgc_port + 9, "QGC_PORT")
    control_port = get_next_available_port(control_port, control_port + 9, "CONTROL_PORT")

    print(f"[INFO] Using QGC port: {qgc_port}")

    # Launch container with environment variables and port mappings
    subprocess.run([
        "docker", "run", "-itd",
        "--name", container_name,
        "-e", f"PX4_HOME_LAT={PX4_HOME_LAT}",
        "-e", f"PX4_HOME_LON={PX4_HOME_LON}",
        "-e", f"PX4_HOME_ALT={PX4_HOME_ALT}",
        "-e", f"QGC_PORT={qgc_port}",
        "-e", f"CONTROL_PORT={control_port}",
        "-p", f"{control_port}:{control_port}/udp",
        "-p", f"{qgc_port}:{qgc_port}/udp",
        DOCKER_IMAGE
    ], check=True)

    print("[INFO] PX4 container starting...")
    return container_name

def get_used_ports_from_px4_containers(start_port, max_port, env_var_name):
    """
    Retrieve a set of used ports by PX4 containers within a given range.
    Considers both port bindings and environment variables.
    """
    result = subprocess.run(["docker", "ps", "--format", "{{.Names}}"], capture_output=True, text=True)
    container_names = [line for line in result.stdout.splitlines() if line.startswith(DOCKER_IMAGE)]
    used_ports = set()

    for name in container_names:
        try:
            inspect = subprocess.check_output(["docker", "inspect", name], text=True)
            container_info = json.loads(inspect)[0]

            # Extract used UDP ports
            ports = container_info.get("NetworkSettings", {}).get("Ports", {})
            for port_proto, bindings in ports.items():
                if bindings and port_proto.endswith("/udp"):
                    for binding in bindings:
                        if binding.get("HostIp") in ["0.0.0.0", LOCAL_IP, "::"]:
                            host_port = int(binding["HostPort"])
                            if host_port >= start_port:
                                used_ports.add(host_port)

            # Extract port from environment variable
            env_vars = container_info.get("Config", {}).get("Env", [])
            for env in env_vars:
                if env.startswith(f"{env_var_name}="):
                    env_port = int(env.split("=")[1])
                    if env_port >= start_port:
                        used_ports.add(env_port)

        except Exception as e:
            print(f"[WARNING] Failed to inspect container {name}: {e}")

    return used_ports

def get_next_available_port(start_port, max_port, env_var_name):
    """
    Find the next available UDP port in the specified range.
    """
    used_ports = get_used_ports_from_px4_containers(start_port, max_port, env_var_name)
    for port in range(start_port, max_port + 1):
        if port not in used_ports:
            return port

    raise RuntimeError(f"No available ports in range for {env_var_name}.")

async def wait_for_heartbeat(master):
    """
    Wait until a MAVLink heartbeat is received from the drone.
    """
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat from sysid={master.target_system}, compid={master.target_component}")

async def arm_drone(master):
    """
    Send arm command to the drone and wait for acknowledgment.
    """
    print("Arming...")
    master.arducopter_arm()
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if msg and msg.result == 0:
        print("Arm ACK:", msg)
    else:
        raise RuntimeError("Failed to arm")

async def wait_for_altitude(master, min_alt=1000):
    """
    Wait until the drone reaches a minimum relative altitude (in mm).
    """
    print(f"Waiting to reach altitude > {min_alt / 1000:.1f} m...")
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg and msg.relative_alt >= min_alt:
            print(f"Reached altitude: {msg.relative_alt / 1000:.2f} m")
            break
        await asyncio.sleep(0.5)

async def set_takeoff_mode(master):
    """
    Set drone mode to TAKEOFF and verify acknowledgment.
    """
    print("Setting mode to TAKEOFF...")
    master.set_mode('TAKEOFF')
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack and ack.result == 0:
        print("Mode change ACK:", ack)
    else:
        raise RuntimeError("Failed to set TAKEOFF mode")

async def set_rtl_mode(master):
    """
    Set drone mode to RTL (Return To Launch) and verify acknowledgment.
    """
    print("Setting mode to RTL...")
    master.set_mode('RTL')
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack and ack.result == 0:
        print("Mode change ACK:", ack)
    else:
        raise RuntimeError("Failed to set RTL mode")

async def send_mission(master, lat, lon, alt):
    """
    Send a simple mission to the drone with one takeoff and one waypoint.
    """
    print("Sending mission...")
    master.waypoint_clear_all_send()

    # Create takeoff and waypoint mission items
    takeoff = mavutil.mavlink.MAVLink_mission_item_message(
        master.target_system, master.target_component,
        seq=0, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, current=0, autocontinue=1,
        param1=0, param2=0, param3=0, param4=0, x=lat, y=lon, z=alt
    )

    waypoint = mavutil.mavlink.MAVLink_mission_item_message(
        master.target_system, master.target_component,
        seq=1, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, current=0, autocontinue=1,
        param1=0, param2=0, param3=0, param4=0, x=lat, y=lon, z=alt
    )

    # Send mission items
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
    """
    Send command to start the uploaded mission.
    """
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
    """
    Wait until the drone reaches the specified mission waypoint.
    """
    print(f"Waiting to reach waypoint #{target_seq}...")
    while True:
        msg = master.recv_match(type='MISSION_ITEM_REACHED', blocking=True, timeout=10)
        if msg and msg.seq == target_seq:
            print(f"[MISSION] Waypoint {msg.seq} reached.")
            break
        await asyncio.sleep(1)

def is_armed(base_mode):
    """
    Check if the drone is currently armed.
    """
    return (base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

async def wait_until_disarmed(master, timeout=60):
    """
    Wait for the drone to disarm within a timeout (seconds).
    """
    print("[INFO] Waiting for drone to disarm...")
    for _ in range(timeout * 2):  # check every 0.5 sec
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and not is_armed(msg.base_mode):
            print("[INFO] Drone is disarmed.")
            return
        await asyncio.sleep(0.5)
    raise TimeoutError("[ERROR] Drone did not disarm within timeout.")

async def main():
    """
    Main orchestration function: launches PX4, connects, arms, flies a mission, then returns and disarms.
    """
    container_name = run_unique_px4_container()

    try:
        master = mavutil.mavlink_connection(f'udp:{LOCAL_IP}:{control_port}')
        await wait_for_heartbeat(master)
        await asyncio.sleep(5)
        await arm_drone(master)
        await send_mission(master, lat=47.397751, lon=8.545607, alt=10)
        await set_takeoff_mode(master)
        await wait_for_altitude(master, min_alt=2)
        await start_mission(master)
        await wait_until_waypoint_reached(master, target_seq=1)
        await set_rtl_mode(master)
        await wait_until_disarmed(master, timeout=60)
        print("[INFO] Mission completed. Drone is returning to launch.")
    finally:
        print(f"[INFO] Cleaning up container: {container_name}")
        subprocess.run(["docker", "stop", container_name])
        subprocess.run(["docker", "rm", container_name])

if __name__ == '__main__':
    asyncio.run(main())