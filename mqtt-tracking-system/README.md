# ArUco Tracker → MQTT Server

This repo contains a small Python server that:

- grabs frames from a camera,
- detects **ArUco markers** (robots),
- converts marker pixel coordinates → **world coordinates (meters)** using a precomputed **homography**,
- publishes all robot states as JSON via **MQTT**,
- optionally shows a warped top-down view for debugging/visualization.

This server was tested on Ubuntu 22 using mosquitto. Using a different operating system may require additional configuration or adjustments.

---

## Quick start

### 1) Install dependencies (once)

```bash
python -m venv .venv
source .venv/bin/activate

pip install numpy pyyaml paho-mqtt opencv-contrib-python
```

> `opencv-contrib-python` is required (the ArUco module is not included in plain `opencv-python`).

### 2) Install and configure Mosquitto (once)

Install Mosquitto and the client tools:

```bash
sudo apt update
sudo apt install mosquitto mosquitto-clients
```

Enable and start the broker:

```bash
sudo systemctl enable mosquitto
sudo systemctl start mosquitto
```
If you want to use the dashboard, you need to modify the mosquitto configuration. Create a custom configuration file:

```bash
sudo nano /etc/mosquitto/conf.d/listeners.conf
```

Add the following content:
```conf
listener 1883
protocol mqtt

listener 8080
protocol websockets

allow_anonymous true
```
> If you want to disable `allow_anonymous`, you need to configure authentication.

### 3) Configure arena and IP address (if setup changes)

Edit `config.yaml`, modifying the arena dimensions and the IP address of your broker, and ensure your MQTT broker is reachable.

### 4) Create a homography (if setup changes)

The server **requires** a valid `homography.npy` (3×3 matrix). If it is missing/empty/invalid, the server will exit and tell you to run your homography creation script.

Generate it using the calibration script in this repo.

```bash
python homography.py
```
Once the image is shown, press `s`. Then, in the new window, click on the four corners of your arena, starting at the lower left corner, the origin `(0,0)`, moving to the lower right, `(arena_width, 0)`, the upper right corner, `(arena_width, arena_height)`, and the upper left corner, `(0, arena_height)`. Press `s` to finalize. The updated homography matrix will be stored in `homography.npy`.

### 5) Run the server (every time)

```bash
python server.py
```

This assumes that the config.yaml file and the homography.npy file are located in the same directory as the script file. To use different configuration paths add them like this:

```bash
python server.py path/to/config.yaml path/to/homography.npy
```

Quit visualization with **q**.

### 6) Run the dashboard (every time)

On the same PC/server that runs the <code>server.py</code>: 

Ensure that <code>node.js</code> is [installed](https://nodejs.org/en/download).  
If the node modules are not yet installed, open a new terminal, navigate to the directory <code>dashboard</code> and run:
```bash
npm install
```

To run the dashoard, open a new terminal, navigate to the directory <code>dashboard</code> and run:
```bash
node server.js
```

Anyone in the network should now be able to access the dashboard by typing <code><mqtt-broker-IP>:3000</code>.

---


## MQTT output

The server publishes a JSON object where each key is an ArUco marker ID and the value is the robot state:

```json
{
  "12": {
    "position": [1.234, 0.456],
    "angle": 273.4
  },
  "7": {
    "position": [0.120, 1.900],
    "angle": 91.0
  }
}
```

- `position` is `[x, y]` in **meters**.
- `angle` is the marker heading in degrees.

The topic is configured via `mqtt.topic`.




## MQTT → ROS 2 Bridge (optional)

If your robotics system uses **ROS 2**, you can run the included bridge node that subscribes to the MQTT topic and republishes each robot as a `geometry_msgs/PoseStamped`.

Each robot gets its own ROS 2 topic: `/poses/<marker_id>` (e.g. `/poses/12`, `/poses/7`).

### Prerequisites

- A working **ROS 2** installation (Humble, Iron, Jazzy, …)
- The `paho-mqtt` and `pyyaml` Python packages (already in `requirements.txt`)

### Run the bridge

```bash
# Source your ROS 2 workspace first
source /opt/ros/<distro>/setup.bash

# Then run the bridge (reads mqtt section from config.yaml)
python3 mqtt_to_ros_bridge.py
```

Or with a custom config path:

```bash
python3 mqtt_to_ros_bridge.py path/to/config.yaml
```

### Verify

In another terminal:

```bash
ros2 topic list          # should show /poses/<id> topics
ros2 topic echo /poses/12   # see live PoseStamped messages
```

---

## Notes

- Marker ID `0` is ignored in the tracking loop.
- Visualization is optional (`visualization.enabled`).

