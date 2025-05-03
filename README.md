## Overview 😊

#### Created by Ilan Mulakandov

This project gives you a complete, ready-to-run **drone simulator** inside Docker on Ubuntu 22.04.

It includes:

- **PX4 SITL** (the flight software that runs in “Software In The Loop” mode)
- **ROS 2** (a common robotics toolkit)
- **Gazebo Classic** (a 3D world physical simulator)
- **mavlink-router** (קxposes a communication path for observing and controlling platform behavior)

With it you can:

1. Pull ("Download") or build the simulator image in seconds
2. Set your own “home” location for the virtual drone
3. Run everything without opening any graphics windows
4. Automate flights with a simple Python script

## 1. What You Need (Prerequisites)

- **Docker Desktop** installed on your computer
- (Optional) A zip file called `px4_sitl.zip` if you want to load a saved simulator image
- **Python 3.8+** and **pip** to run the example flight script

## 2. Getting the Simulator Image

### 2.1 Pull ("Download") from Docker Hub

Just type this in your command line:

```bash
docker pull ilanmulakandov/px4_sitl
```

This always grabs the latest version.
For checking you have it, write this command at your command line:

```bash
docker images | grep px4_sitl
```

### 2.2 Build Yourself (Optional)

If you’d rather build the image from scratch:

```bash
docker build \
  --build-arg BASE_IMAGE=ubuntu \
  --build-arg TAG=22.04 \
  -t px4_sitl .
```

**Note:** It is recommended to stick with Ubuntu 22.04 for best results.
But if you want to change, you can, and it is recommended to check the integrity of the docker afterwards before any significant operation.

### 2.3 Load from Archive (Optional)

If you have a `px4_sitl.zip` file:

1. Unzip it to find `px4_sitl.tar.gz`.
2. In your terminal, `cd` into the directory containing that file.
3. Run:

   ```bash
   docker load --input px4_sitl.tar.gz
   ```

4. Verify with:

   ```bash
   docker images | grep px4_sitl
   ```

**Tip:** To save your own image as a ZIP, use:

```bash
docker save -o px4_sitl.tar.gz px4_sitl:latest
```

## 3. Running the Simulator independently

Start the simulator (headless) with:

```bash
docker run --rm -it \
  -p 14540:14540/udp \
  -p 14550:14550/udp \
  -e PX4_HOME_LAT=47.3977 \
  -e PX4_HOME_LON=8.5456 \
  -e PX4_HOME_ALT=488.0 \
  px4_sitl
```

And what actually happens when you run this command?

- **mavlink-router** will launch in the background for data routing.
- Your chosen home coordinates are applied before the simulator starts (**PX4_HOME** variables).
- PX4 SITL launches with Gazebo Classic automatically.
- Since port **14550** is open, you can point _**QGroundControl**_ (Real-time drone monitoring app) at UDP port **14550** and immediately see the drone’s location and status-no extra setup needed.

> **Note:** If you use the `run_mission.py` script, Each new container may shift ports (e.g., 14551, 14552…) to avoid conflicts. You can adjust this range in the script and the port at QGC for visualization.

## 4. Custom Settings

You can tweak these values when you run the container:

| Name           | Default     | What It Means                              |
| -------------- | ----------- | ------------------------------------------ |
| `PX4_HOME_LAT` | `47.397751` | Starting latitude of your virtual drone    |
| `PX4_HOME_LON` | `8.545607`  | Starting longitude of your virtual drone   |
| `PX4_HOME_ALT` | `488.13123` | Starting altitude (in meters)              |
| `QGC_IP`       | `10.0.0.16` | IP address for QGroundControl (your PC)    |
| `QGC_PORT`     | `14550`     | UDP port for QGroundControl                |
| `CONTROL_IP`   | `10.0.0.16` | IP address for any custom control software |
| `CONTROL_PORT` | `14540`     | UDP port for custom control software       |

## 5. Runing Automating Flights

### 5.1 `run_mission.py`

This Python script will:

1. Launch a new PX4 container with a unique name
2. Connect over UDP to the simulator
3. Wait for the “heartbeat,” then arm the drone
4. Send it a simple mission (takeoff + one waypoint)
5. Switch to **TAKEOFF** then **RTL** (return-to-launch) mode
6. Stop and remove the container when done

> **Tip:** Edit the script’s top section to match your IP, ports, and home coordinates.

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

## 6. Why ROS 2 Is Included 🚀

Even if you don’t use ROS 2 directly, having it in the same container means:

1. You can run tools like **rviz2** or **ros2 topic echo** without extra setup.
2. Developers who build advanced behaviors (SLAM, navigation) can start immediately.
3. All dependencies live in one place-no version clashes.

This all-in-one setup keeps development smooth and simulation flexible-making it easy to get started. 😄

## 7. Best Practices & Tips

- **Set home first:** Always pass your `PX4_HOME_*` values before launching.
- **Unique container names**: Prevent collisions in parallel runs.
- **Auto-cleanup:** `--rm` makes Docker delete the container when it stops.
- **Offline builds:** Use `docker save`/`load` to share images without internet.

Happy flying! 🛩️
