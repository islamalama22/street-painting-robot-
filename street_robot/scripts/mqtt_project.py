#!/usr/bin/env python3
import time
import paho.mqtt.client as paho
import threading
import random
import rospy
from std_msgs.msg import String, Bool

# Global Publisher for Emergency Stop
emergency_stop_pub = None

# Global Variables
data_received = {}
stop_case = "no"

# MQTT Broker Configuration
BROKER = "broker.hivemq.com"
PORT = 1883

# MQTT Topics
MQTT_TOPIC_SUBSCRIBE = "ppu/-pro--->draw/line"
MQTT_TOPIC_ALERT = "ppu/-pro--->color/intensity"
MQTT_FINISH_DRAWING = "ppu/-pro--->finish/drawing"
MQTT_OBSTACLE_FOUND = "ppu/-pro--->found/obstacle"
MQTT_STOP_TOPIC = "ppu/-pro--->stop"

# MQTT Callback Functions
def on_connect(client, userdata, flags, rc, properties=None):
    print("CONNACK received with code %s." % rc)
    if rc == 0:
        print("Connected to MQTT Broker!")
    else:
        print("Failed to connect, return code %d\n" % rc)

def on_publish(client, userdata, mid, properties=None):
    print("Publish OK => mid: " + str(mid))

def on_subscribe(client, userdata, mid, granted_qos, properties=None):
    print("Subscribed OK => mid: " + str(mid) + " " + str(granted_qos))

def on_message(client, userdata, msg):
    topic = str(msg.topic)
    payload = str(msg.payload.decode('utf-8'))
    print(f"Message received on topic {topic}: {payload}")
    process_message(topic, payload)

# MQTT Client Setup
client_name_id = "client_" + str(random.randint(1, 100)) + "_" + str(random.randint(1, 100))
client = paho.Client(client_id=client_name_id, userdata=None, protocol=paho.MQTTv31)

# Assign MQTT Callback Functions
client.on_connect = on_connect
client.on_subscribe = on_subscribe
client.on_message = on_message
client.on_publish = on_publish

# Connect to MQTT Broker
try:
    client.connect(BROKER, PORT)
    client.subscribe(MQTT_TOPIC_SUBSCRIBE, qos=1)
    client.subscribe(MQTT_STOP_TOPIC, qos=1)  # Subscribe to the stop topic
except Exception as e:
    print(f"Failed to connect to MQTT Broker: {e}")

# Function to Publish MQTT Messages
def publish_mqtt(topic, msg):
    try:
        client.publish(topic, payload=msg, qos=1)
    except Exception as e:
        print(f"Error publishing to MQTT: {e}")

# Function to Process Received Messages
def process_message(topic, msg):
    global data_received, stop_case

    if topic == MQTT_TOPIC_SUBSCRIBE:
        try:
            data = {key_value.split(":")[0].strip(): key_value.split(":")[1].strip() for key_value in msg.split(",")}
            print("Parsed data:", data)
            data_received = data

            # Save parsed data to a Python file
            with open("/home/neobotix/ros_workspace/src/street_robot/scripts/mqtt_data.py", "w") as f:
                f.write(f"STREET_LENGTH = {data.get('street_length', 'None')}\n")
                f.write(f"STREET_WIDTH = {data.get('street_width', 'None')}\n")
                f.write(f"LINE_NUMBER = {data.get('line_number', 'None')}\n")
                f.write(f"LINE_LENGTH = {data.get('line_length', 'None')}\n")
                f.write(f"LINE_COLOR = '{data.get('line_color', 'Unknown')}'\n")
                f.write(f"DISTANCE_BETWEEN_LINES = {data.get('distance_between_lines', 'None')}\n")
                f.write(f"LEFT_RIGHT_MOVEMENT = {data.get('left_right_movement', 'None')}\n")
                f.write(f"FRONT_BACK_MOVEMENT = {data.get('front_back_movement', 'None')}\n")
                f.write(f"LINE_DIRECTION = '{data.get('line_direction', 'Unknown')}'\n")
            print("Data saved to mqtt_data.py")
        except Exception as e:
            print(f"Error processing message: {e}")

    if topic == MQTT_STOP_TOPIC:  # Handle stop topic
        try:
            stop_case = msg.lower()
            print(f"stop_case: {stop_case}")
            
            if stop_case == "true":
                print("Emergency stop triggered via MQTT.")
                rospy.loginfo("Publishing emergency stop to /relayboard_v2/emergency_stop_state.")
                emergency_stop_pub.publish(Bool(data=True))  # Publish True to trigger emergency stop
        except Exception as e:
            print(f"Error processing stop message: {e}")

# ROS Ultrasonic Sensor Callback
def ultrasonic_callback(msg):
    """
    Publish to MQTT if the ultrasonic sensor detects low paint level.
    """
    if msg.data:  # Ultrasonic indicates low paint level
        try:
            publish_mqtt(MQTT_TOPIC_ALERT, "low level of paint")
        except Exception as e:
            rospy.logerr(f"Error publishing to MQTT: {e}")

# Start MQTT Client Loop
threading.Thread(target=lambda: client.loop_forever(), daemon=True).start()

# ROS Node Main Function
def main():
    global emergency_stop_pub

    rospy.init_node("mqtt_project", anonymous=True)
    publish_mqtt(MQTT_OBSTACLE_FOUND , " no obstacle")
    publish_mqtt(MQTT_TOPIC_ALERT, " the paint level is suitable")
    # ROS Publishers and Subscribers
    emergency_stop_pub = rospy.Publisher('/relayboard_v2/emergency_stop_state', Bool, queue_size=10)
    rospy.Subscriber('/ultrasonic_read_bool', Bool, ultrasonic_callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

