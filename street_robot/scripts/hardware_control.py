#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
import serial
import threading
import paho.mqtt.client as mqtt

# Global variables
arduino_serial = None
valve_state_pub = None
relay_state_pub = None
servo_angle_pub = None
ultrasonic_pub = None
mqtt_client = None

#####################################  2 ##############################
def publish_mqtt(topic, message):
    """
    Publish a message to an MQTT topic.
    """
    try:
        mqtt_client.publish(topic, message)
        rospy.loginfo(f"Published to MQTT topic {topic}: {message}")
    except Exception as e:
        rospy.logerr(f"Failed to publish to MQTT: {e}")


def read_arduino():
    """
    Thread to continuously read status updates from the Arduino and publish them as ROS topics.
    """
    global arduino_serial

    while not rospy.is_shutdown():
        try:
            if arduino_serial.in_waiting > 0:
                status = arduino_serial.readline().decode('utf-8').strip()
                rospy.loginfo(f"Arduino status: {status}")

                # Parse ultrasonic readings from warning messages
                if "WARNING: Painting level is LOW! Distance:" in status:
                    try:
                        # Extract distance value from the message
                        distance_str = status.split("Distance:")[-1].strip().split(" ")[0]
                        distance = float(distance_str)
                        less_than_15 = distance < 15
                        ultrasonic_pub.publish(less_than_15)  # Publish to ROS topic

                        rospy.loginfo(f"Ultrasonic distance: {distance} cm, Less than 15: {less_than_15}")
                    except (ValueError, IndexError) as e:
                        rospy.logwarn(f"Failed to parse distance from status: {status}. Error: {e}")

                # Parse other status updates and publish to appropriate topics
                elif "Valve opened" in status:
                    valve_state_pub.publish("OPEN")
                    relay_state_pub.publish("ON")
                    servo_angle_pub.publish("10")
                elif "Valve closed" in status:
                    valve_state_pub.publish("CLOSED")
                    relay_state_pub.publish("OFF")
                    servo_angle_pub.publish("50")
        except Exception as e:
            rospy.logerr(f"Error reading from Arduino: {e}")

###################################### 4 ####################################################

def hardware_run_callback(data):
    """
    Callback for the 'hardware_run' topic.
    Sends a command to Arduino to open the valve, activate the relay, and set the servo to 120°.
    """
    rospy.loginfo("Received 'hardware_run'. Sending 'open' command to Arduino.")
    if arduino_serial:
        arduino_serial.write("open\n".encode())


def hardware_stop_callback(data):
    """
    Callback for the 'hardware_stop' topic.
    Sends a command to Arduino to close the valve, deactivate the relay, and reset the servo to 0°.
    """
    rospy.loginfo("Received 'hardware_stop'. Sending 'close' command to Arduino.")
    if arduino_serial:
        arduino_serial.write("close\n".encode())

#####################################  5 #################################3
def setup_mqtt():
    """
    Set up the MQTT client.
    """
    global mqtt_client

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            rospy.loginfo("Connected to MQTT Broker!")
        else:
            rospy.logerr(f"Failed to connect to MQTT Broker. Return code: {rc}")

    mqtt_client = mqtt.Client(protocol=mqtt.MQTTv5)
    mqtt_client.on_connect = on_connect

    try:
        mqtt_client.connect("localhost", 1883, 60)  # Update host/port if needed
        threading.Thread(target=lambda: mqtt_client.loop_forever(), daemon=True).start()
    except Exception as e:
        rospy.logerr(f"Failed to connect to MQTT Broker: {e}")


#####################################  6 #################################3
def main():
    global arduino_serial, valve_state_pub, relay_state_pub, servo_angle_pub, ultrasonic_pub

    rospy.init_node('hardware_control', anonymous=True)

    # Initialize serial communication with Arduino
    try:
        arduino_serial = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Update port as needed
        rospy.loginfo("Connected to Arduino via serial.")
    except serial.SerialException as e:
        rospy.logerr(f"Failed to connect to Arduino: {e}")
        return

    # Initialize MQTT
    setup_mqtt()

    # Initialize publishers for hardware state
    valve_state_pub = rospy.Publisher('/hardware/valve_state', String, queue_size=10)
    relay_state_pub = rospy.Publisher('/hardware/relay_state', String, queue_size=10)
    servo_angle_pub = rospy.Publisher('/hardware/servo_angle', String, queue_size=10)
    ultrasonic_pub = rospy.Publisher('/ultrasonic_read_bool', Bool, queue_size=10)

    # Subscribers for hardware control commands
    rospy.Subscriber('hardware_run', String, hardware_run_callback)
    rospy.Subscriber('hardware_stop', String, hardware_stop_callback)

    # Start thread to read from Arduino
    threading.Thread(target=read_arduino, daemon=True).start()

    rospy.loginfo("Hardware control node initialized. Waiting for commands...")

    rospy.spin()

    # Close the serial connection on exit
    if arduino_serial:
        arduino_serial.close()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("Hardware control node terminated.")

