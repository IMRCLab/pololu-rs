import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck # Unly needed for the Pipuck

# Define variables and callbacks
Broker = "000,000.000.000"  # Replace with your broker address
Port = 1883 # MQTT port
Topic = "" # Replace with your own topic
robot_data = None # global variable to store robot data

# function to handle connection
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe(Topic)

# function to handle incoming messages
def on_message(client, userdata, msg):
    global robot_data
    try:
        robot_data = json.loads(msg.payload.decode())
    except json.JSONDecodeError:
        print(f'invalid json: {msg.payload}')

# Initialize MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(Broker, Port, 60)

client.loop_start() # Start listening loop in separate thread

# Initialize the PiPuck
pipuck = PiPuck(epuck_version=2)

# Set the robot's speed, e.g. with
pipuck.epuck.set_motor_speeds(1000,-1000)

for _ in range(1000):
    # TODO: Do your stuff here
    time.sleep(1)
    
# Stop the MQTT client loop
client.loop_stop()  
