## Overview 😊

This repository provides a **containerized PX4 SITL** environment **with ROS 2** on Ubuntu 22.04, complete with **Gazebo Classic** and **mavlink-router**. It lets you:

1. Quickly build or load a prebuilt Docker image
2. Override the simulated drone’s home position via environment variables
3. Run headless SITL + Gazebo with all MAVLink ports exposed
4. Automate missions using a companion Python script with **pymavlink**

---

## 1. Prerequisites

- Docker (Desktop or Engine) installed
- (Optional) `px4_docker.zip` if you want to load a saved image
- Python 3.8+ and `pip` for running the mission script

---

## 2. Building & Loading the Docker Image

### 2.1 Build from Source

```bash
docker build \
  --build-arg BASE_IMAGE=ubuntu \
  --build-arg TAG=22.04 \
  -t px4_sitl .
```

### 2.2 Load from Archive

If you have a `px4_docker.zip` archive:

1. Unzip it anywhere. You will find `px4_docker.tar.gz` (or `.tar`).
2. In your terminal, `cd` to the directory containing that file.
3. Load the image into Docker:

   ```bash
   docker load --input px4_docker.tar.gz
   ```

After this, you should see `px4_sitl:latest` listed when you run:

```bash
docker images | grep px4_sitl
```

> **Tip:** To create your own tarball from a local image, run:
>
> ```bash
> docker save -o px4_docker.tar.gz px4_sitl:latest
> ```

---

## 3. Running the PX4 SITL Container

```bash
docker run --rm -it \
  -p 14540:14540/udp \
  -p 14550:14550/udp \
  -e PX4_HOME_LAT=47.3977 \
  -e PX4_HOME_LON=8.5456 \
  -e PX4_HOME_ALT=488.0 \
  px4_sitl
```

1. **mavlink-routerd** starts in the background, forwarding MAVLink traffic to any endpoints.
2. Environment variables for **PX4_HOME\_**\* are exported **before** SITL starts.
3. PX4 SITL launches with Gazebo Classic automatically.
4. Because UDP port **14550** is exposed for _`QGroundControl`_ (-p 14550:14550/udp), you can simply open QGC on your host and connect to **UDP** port **14550** to view the simulated vehicle’s telemetry and position—no additional port remapping needed.

   **Note:** If you use run_mission script, this port will be changed according to the existing and running of the same containers in your Docker Desktop. It will go to 14551,14552, and so on until 14559 (you can change the range inside the script).

---

## 4. Configurable Environment Variables

| Variable       | Default     | Description                      |
| -------------- | ----------- | -------------------------------- |
| `PX4_HOME_LAT` | `47.397751` | Home latitude (decimal degrees)  |
| `PX4_HOME_LON` | `8.545607`  | Home longitude (decimal degrees) |
| `PX4_HOME_ALT` | `488.13123` | Home altitude (meters MSL)       |
| `QGC_IP`       | `10.0.0.16` | QGroundControl IP                |
| `QGC_PORT`     | `14550`     | QGroundControl UDP port          |
| `CONTROL_IP`   | `10.0.0.16` | Custom control software IP       |
| `CONTROL_PORT` | `14540`     | Custom control software port     |

---

## 5. Automated Mission Script

### 5.1 `run_mission.py`

This asyncio‑based Python script will:

1. Launch a uniquely named PX4 container
2. Connect via **pymavlink** over UDP
3. Wait for heartbeat and arm the vehicle
4. Send a mission (takeoff + waypoint)
5. Switch to **TAKEOFF** and then **RTL** mode
6. Clean up the container when finished

**Note:** you must change all variables to ones that suit your desires and computer (pay close attention to changing the variables that are marked as needing to be changed)

```python
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

# … rest of script …
```

### 5.2 `requirements.txt`

```text
pymavlink>=2.4.15
```

Install with:

```bash
pip install -r requirements.txt
```

---

## 6. Why ROS 2 Is Included 🚀

Including **ROS 2** in this container offers:

1. **Full ROS 2 Integration**: Run `rviz2`, `ros2 topic echo /fcu/gps` or `ros2 run mavros mavsys mode -c OFFBOARD` alongside PX4 and Gazebo.
2. **Rapid Development & Debugging**: Use `rqt_graph`, `ros2 param`, and `rosbag2` without bridging multiple containers.
3. **Unified Dependencies**: One image bundles PX4, Gazebo, mavlink-router, and a fixed ROS 2 distro (Galactic/Humble).
4. **Ready for Advanced Workflows**: Add SLAM, navigation2, sensor drivers, or other ROS 2 stacks without rebuilding.

This all‑in‑one design keeps your CI pipeline lean, development smooth, and simulation flexible. 😄

---

## 7. Best Practices & Tips

- **Export env first**: Ensures `PX4_HOME_*` are applied correctly.
- **Unique container names**: Prevent collisions in parallel runs.
- **Use `--rm`**: Automatically cleans up stopped containers.
- **Air‑gapped CI**: Leverage `docker save`/`load` for reproducible builds without internet.

Happy flying! 🛩️
