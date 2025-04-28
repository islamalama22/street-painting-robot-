#!/usr/bin/env python3
#######################333 
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
import math
import time
import threading  # Add this import
import os  # Import os for system calls
import paho.mqtt.client as paho
from actionlib_msgs.msg import GoalStatusArray, GoalID
from threading import Lock

last_print_time = 0
print_count = 0
rate_limit_interval = 12  # Interval in seconds for 5 prints per minute


# Dictionary to store goal_id and its status
goal_status_dict = {}
lock = Lock()  # To ensure thread-safe updates


comparison_ready = threading.Event()  # Initialize the threading event

# Import data from mqtt_data.py
from mqtt_data import (
    STREET_LENGTH, STREET_WIDTH, LINE_NUMBER, LINE_LENGTH, 
    LINE_COLOR, DISTANCE_BETWEEN_LINES, LEFT_RIGHT_MOVEMENT, 
    FRONT_BACK_MOVEMENT, LINE_DIRECTION
)

# Parameters
MAX_RETRIES = 2
SAFE_DISTANCE = 0.5  # Minimum distance from obstacles

# Global variables
x_current = 0.0
y_current = 0.0
obstacle_distance = float('inf')

# ROS Publishers
hardware_run_pub = None
hardware_stop_pub = None
##################################    mqtt    4  #################################################

BROKER = "broker.hivemq.com"
PORT = 1883

data_received = {}
stop_case = "no"

# Topics
MQTT_TOPIC_SUBSCRIBE = "ppu/-pro--->draw/line"
MQTT_TOPIC_ALERT = "ppu/-pro--->color/intensity"
MQTT_FINISH_DRAWING = "ppu/-pro--->finish/drawing"
MQTT_OBSTACLE_FOUND = "ppu/-pro--->found/obstacle"

# MQTT Client Setup
import paho.mqtt.client as paho
import threading

client = paho.Client()


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        rospy.loginfo("Connected to MQTT Broker!")
    else:
        rospy.logwarn(f"Failed to connect to MQTT Broker. Code: {rc}")

client.on_connect = on_connect

try:
    client.connect(BROKER, PORT)
    threading.Thread(target=lambda: client.loop_forever(), daemon=True).start()
except Exception as e:
    rospy.logerr(f"Failed to connect to MQTT Broker: {e}")



# Function to Publish MQTT Messages
def publish_mqtt(topic, msg):
    try:
        client.publish(topic, payload=msg, qos=1)
    except Exception as e:
        rospy.logerr(f"Error publishing to MQTT: {e}")


def on_publish(client, userdata, mid, properties=None):
    print("Publish OK => mid: " + str(mid))

def on_subscribe(client, userdata, mid, granted_qos, properties=None):
    print("Subscribed OK => mid: " + str(mid) + " " + str(granted_qos))

def on_message(client, userdata, msg):
    topic = str(msg.topic)
    payload = str(msg.payload.decode('utf-8'))
    print(f"Message received on topic {topic}: {payload}")
    process_message(topic, payload)
 


######################################  7  setup  ######################################################
def current_pose_callback(data):
    global x_current, y_current
    x_current = data.pose.pose.position.x
    y_current = data.pose.pose.position.y

def laser_callback(data):
    global obstacle_distance
    obstacle_distance = min(data.ranges)

def clear_costmaps():
    rospy.wait_for_service('/move_base/clear_costmaps')
    try:
        clear_costmap_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        clear_costmap_service()
        rospy.loginfo("Costmaps cleared to refresh path planning.")
    except rospy.ServiceException as e:
        rospy.logwarn(f"Failed to clear costmaps: {e}")



# Global variables for rate limiting
last_yes_time = 0  # Timestamp of the last "Obstacle detected" print
last_no_time = 0   # Timestamp of the last "No obstacle detected" print
rate_limit_interval = 20  # Minimum interval in seconds (3 messages per minute)

from actionlib_msgs.msg import GoalID

# Initialize a publisher for canceling goals
cancel_goal_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

from actionlib_msgs.msg import GoalID

# Global variable to keep track of the current goal index
current_goal_index = 0
goal_lines = []  # Holds the list of all goals (lines)


def obst_detect_callback(msg):
    
    global last_yes_time, current_goal_index, goal_lines, cancel_goal_pub, hardware_stop_pub

    current_time = rospy.Time.now().to_sec()

    if msg.data == "yes":  # Obstacle detected
        if current_time - last_yes_time >= rate_limit_interval:
            rospy.logwarn("Obstacle detected! Canceling the current goal.")
            last_yes_time = current_time

            # Publish MQTT message for obstacle detection
            publish_mqtt(MQTT_OBSTACLE_FOUND, f"Obstacle detected at Line {current_goal_index + 1}. Canceling goal.")

            # Cancel the current goal and stop hardware
            cancel_msg = GoalID()
            cancel_goal_pub.publish(cancel_msg)
            hardware_stop_pub.publish("stop")  # Stop hardware immediately
            rospy.loginfo("Current goal canceled. Hardware stopped.")

            # Move to the next goal only
            current_goal_index += 1
            if current_goal_index < len(goal_lines):
                start, end = goal_lines[current_goal_index]
                rospy.loginfo(f"Processing next goal: Line {current_goal_index + 1} Start={start}, End={end}")

                # Move to the start of the next line
                if not move_with_obstacle_check(start[0], start[1], "Forward", f"Line {current_goal_index + 1} Start"):
                    rospy.logwarn(f"Skipping Line {current_goal_index + 1} Start due to obstacles.")
                    publish_mqtt(MQTT_OBSTACLE_FOUND, f"Skipping Line {current_goal_index + 1} Start due to obstacles.")
                    return

                # Explicitly open hardware at the start of the next line
                rospy.loginfo(f"Reached Start of Line {current_goal_index + 1}. Opening hardware.")
                update_hardware_state(open_valve=True)
                rospy.sleep(0.2)

                # Move to the end of the next line
                if not move_with_obstacle_check(end[0], end[1], "Forward", f"Line {current_goal_index + 1} End"):
                    rospy.logwarn(f"Skipping Line {current_goal_index + 1} End due to obstacles.")
                    update_hardware_state(open_valve=False)
                    publish_mqtt(MQTT_OBSTACLE_FOUND, f"Skipping Line {current_goal_index + 1} End due to obstacles.")
                    return

                # Close hardware after completing the line
                rospy.loginfo(f"Line {current_goal_index + 1} completed. Closing hardware.")
                update_hardware_state(open_valve=False)
                rospy.sleep(0.2)

                # Publish MQTT message for line completion
                publish_mqtt(MQTT_FINISH_DRAWING, f"Line {current_goal_index + 1} completed successfully.")
            else:
                rospy.loginfo("No remaining goals to process.")
                publish_mqtt(MQTT_FINISH_DRAWING, "All lines processed successfully.")
    else:
        # No obstacle detected; no further action needed
        pass

def send_lines_as_goals(lines, direction):
    global current_goal_index, goal_lines

    if not lines:
        rospy.logwarn("No lines to process. Exiting.")
        return

    goal_lines = lines
    current_goal_index = 0

    while current_goal_index < len(goal_lines):
        start, end = goal_lines[current_goal_index]
        rospy.loginfo(f"Processing Line {current_goal_index + 1}: Start={start}, End={end}")

        # Move to the start of the line
        if not move_with_obstacle_check(start[0], start[1], direction, f"Line {current_goal_index + 1} Start"):
            rospy.logwarn(f"Skipping Line {current_goal_index + 1} due to issues reaching the start point.")
            update_hardware_state(open_valve=False)  # Ensure hardware is stopped
            current_goal_index += 1
            continue

        # Open hardware at the start of each line
        rospy.loginfo(f"Reached Start of Line {current_goal_index + 1}. Opening hardware.")
        update_hardware_state(open_valve=True)
        rospy.sleep(0.2)

        # Move to the end of the line
        if not move_with_obstacle_check(end[0], end[1], direction, f"Line {current_goal_index + 1} End"):
            rospy.logwarn(f"Skipping Line {current_goal_index + 1} due to obstacles along the line.")
            update_hardware_state(open_valve=False)  # Stop hardware
            current_goal_index += 1
            continue

        # Close hardware after completing the line
        rospy.loginfo(f"Line {current_goal_index + 1} completed. Closing hardware.")
        update_hardware_state(open_valve=False)
        rospy.sleep(0.2)

        current_goal_index += 1

    rospy.loginfo("All lines processed.")


def move_with_obstacle_check(x_target, y_target, direction, description):

    global obstacle_distance, x_current, y_current, cancel_goal_pub

    rospy.loginfo(f"--- Moving to {description}: Target = (x: {x_target}, y: {y_target}) ---")
    send_goal(x_target, y_target, 90 if direction == 'Width' else 0)

    while True:
        rospy.sleep(0.1)

        # Check for obstacles directly in the path
        if obstacle_distance < 1.0:  # Adjust threshold as needed
            rospy.logwarn(f"Obstacle detected while moving to {description}. Canceling...")
            cancel_msg = GoalID()
            cancel_goal_pub.publish(cancel_msg)  # Cancel current goal
            return False

        # Check if the robot has reached the goal
        distance_to_goal = math.sqrt((x_current - x_target) ** 2 + (y_current - y_target) ** 2)
        if distance_to_goal < 0.5:  # Success threshold
            rospy.loginfo(f"--- Successfully reached {description} ---")
            return True



########################################  8 send goales ##############################################3
def send_goal(x_target, y_target, orientation, is_line_start=False, is_line_end=False):
    global x_current, y_current

    rospy.loginfo(f"Sending goal to: x = {x_target:.2f}, y = {y_target:.2f}, orientation = {orientation}")

    # Publish hardware state
    if is_line_start:
        hardware_run_pub.publish("start")
        rospy.loginfo("Hardware state: RUN")
    elif is_line_end:
        hardware_stop_pub.publish("stop")
        rospy.loginfo("Hardware state: STOP")

    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = x_target
    goal.pose.position.y = y_target
    goal.pose.orientation.z = math.sin(math.radians(orientation) / 2)
    goal.pose.orientation.w = math.cos(math.radians(orientation) / 2)

    goal_pub.publish(goal)

    # Monitor progress
    start_time = time.time()
    while time.time() - start_time < 10:
        distance_to_goal = math.sqrt((x_current - x_target)**2 + (y_current - y_target)**2)
        #rospy.loginfo(f"Distance to goal: {distance_to_goal:.2f} meters")
        if distance_to_goal < 0.5:  # Success threshold
            rospy.loginfo("Goal reached.")
            return True
        rospy.sleep(1)

    rospy.logwarn("Failed to reach the goal within the time limit.")
    return False

#########################################
def cancel_current_goal():
    global current_goal_id

    if current_goal_id:  # Corrected indentation
        print(f"\033[93mCancelling current goal with ID: {current_goal_id}\033[0m")
        cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        rospy.sleep(1)
        
        cancel_msg = GoalID()
        cancel_msg.id = current_goal_id
        
        cancel_pub.publish(cancel_msg)
        print(f"\033[92mSuccessfully cancelled goal with ID: {current_goal_id}\033[0m")
        current_goal_id = None
    else:
        print(f"\033[91mNo active goal to cancel.\033[0m")

def status_callback(msg):
    """
    Callback function for monitoring goal status.
    """
    global last_print_time, print_count

    current_time = time.time()

    if not msg.status_list:
        rospy.logwarn("No active goals detected in status list.")
        return

    for status in msg.status_list:
      #  rospy.loginfo_throttle(1, f"Goal Status: ID={status.goal_id.id}, Status={status.status}")
        if status.status == 1:  # Status 1 means "active"
            current_goal_id = status.goal_id.id
            if current_time - last_print_time >= rate_limit_interval or print_count < 5:
              #  rospy.loginfo(f"Active Goal ID detected: {current_goal_id}")
                last_print_time = current_time
                print_count += 1

    ############################################  9     Calculates the initial starting point.  #######################################

def calculate_start_of_first_line():

    global x_current, y_current

    y_target = y_current + FRONT_BACK_MOVEMENT  # Example movement along Y
    if not send_goal(x_current, y_target, 0, is_line_start=False, is_line_end=True):
        rospy.logerr("Failed to reach the first Y goal.")
        return None

    x_target = x_current + LEFT_RIGHT_MOVEMENT  # Example movement along X
    if not send_goal(x_target, y_target, 90, is_line_start=False, is_line_end=True):
        rospy.logerr("Failed to reach the first X goal.")
        return None

    rospy.loginfo(f"Start of first line: x = {x_target:.2f}, y = {y_target:.2f}")
    return x_target, y_target
    
    #######################################   10  lines start and  end point caluation  ##############################################

def calculate_lines(start_x, start_y, line_length, line_spacing, num_lines, direction):
    lines = []
    current_x, current_y = start_x, start_y

    for i in range(1, num_lines + 1):
        if direction == 'Length':  # Horizontal movement
            if i == 1:
                # First line
                start = (current_x, current_y)
                end = (current_x + line_length, current_y)  # Move along X-axis
            elif i == 3:
                # Third line: Start is end of line 1 + 2 * spacing, End is start - line_length
                start = (lines[0][1][0], lines[0][1][1] + 2 * line_spacing)  # End of line 1 + 2 * spacing
                end = (start[0] - line_length, start[1])  # Start minus line_length
            elif i == 4:
                # Fourth line: Start is start of line 2 + 2 * spacing, End is start - line_length
                start = (lines[1][0][0], lines[1][0][1] + 2 * line_spacing)  # Start of line 2 + 2 * spacing
                end = (start[0] - line_length, start[1])  # Start minus line_length
            elif i % 2 == 0:
                # Even lines: Move back along X-axis and up in Y-axis
                start = (lines[-1][1][0] - 1.3, lines[-1][1][1] + line_spacing)  # Move up in Y-axis
                end = (start[0] - line_length, start[1])  # Move back along X-axis
            else:
                # Odd lines (other than line 1): Move forward along X-axis and up in Y-axis
                start = (lines[-1][1][0], lines[-1][1][1] + line_spacing)  # Move up in Y-axis
                end = (start[0] + line_length, start[1])  # Move forward along X-axis
        
        elif direction == 'Width':  # Vertical movement
            if i == 1:
                start = (current_x, current_y)
                end = (current_x, current_y + line_length)  # Move along Y-axis
            elif i % 2 == 0:
                start = (lines[-1][1][0] + line_spacing, lines[-1][1][1])  # Move right in X-axis
                end = (start[0], start[1] - line_length)  # Move back along Y-axis
            else:
                start = (lines[-1][1][0] + line_spacing, lines[-1][1][1])  # Move right in X-axis
                end = (start[0], start[1] + line_length)  # Move forward along Y-axis
        else:
            rospy.logwarn("Invalid direction. Stopping calculation.")
            break
        
        # Append the start and end points for the line
        lines.append((start, end))
    
    for idx, (start, end) in enumerate(lines, 1):
        print(f"Line {idx}: Start {start}, End {end}")
    
    return lines
 
    #######################################    11 save current postion in file    #################################################
def save_current_position(filename):
    global x_current, y_current
    try:
        with open(filename, 'w') as file:  # Overwrite the file
            file.write(f"Current Position: ({x_current:.2f}, {y_current:.2f})\n")
        rospy.loginfo(f"Current Position saved to file: ({x_current:.2f}, {y_current:.2f})")
    except IOError as e:
        rospy.logerr(f"Failed to save current position: {e}")
        
        
############################################    12   start point clacu  ########################################

def calculate_start_point():
    global x_current, y_current
    start_x = x_current + LEFT_RIGHT_MOVEMENT
    start_y = y_current + FRONT_BACK_MOVEMENT
    rospy.loginfo(f"Calculated Start Point: ({start_x:.2f}, {start_y:.2f})")
    return start_x, start_y

############################################    13 print    ########################################

def print_lines(lines):
    for i, (start, end) in enumerate(lines, 1):
        rospy.loginfo(f"Line {i}: Start {start}, End {end}")


# Flags to track hardware state
hardware_open = False  # Indicates if the valve is open
servo_position = "CLOSED"  # Tracks servo state: "OPEN" or "CLOSED"


##################################################  14  update the  hardwer     #####################################
def update_hardware_state(open_valve):
    global hardware_open, servo_position

    if open_valve and not hardware_open:
        hardware_run_pub.publish("start")
        rospy.loginfo("Valve and servo: OPEN")
        hardware_open = True
        servo_position = "OPEN"
    elif not open_valve and hardware_open:
        hardware_stop_pub.publish("stop")
        rospy.loginfo("Valve and servo: CLOSED")
        hardware_open = False
        servo_position = "CLOSED"

        
        
######################################     15  goales sent     #########################################


 ##################################################   19  comper the  points   ############################# 
 
def run_comparison():
    rospy.loginfo("Waiting for line data to be ready...")
    comparison_ready.wait()  # Wait for the main thread's signal
    rospy.loginfo("Starting comparison...")
    try:
        os.system("rosrun street_robot compare_robot_movement.py")
    except Exception as e:
        rospy.logerr(f"Failed to run comparison script: {e}")


def read_saved_lines_safe(filename):
    if not os.path.exists(filename):
        rospy.logerr(f"File not found: {filename}")
        return []

    try:
        with open(filename, 'r') as file:
            lines = []
            for line in file:
                if "Start" in line and "End" in line:
                    try:
                        start_str = line.split("Start")[1].split(", End")[0].strip(" ()\n")
                        end_str = line.split("End")[1].strip(" ()\n")
                        start_tuple = tuple(map(float, start_str.split(", ")))
                        end_tuple = tuple(map(float, end_str.split(", ")))
                        lines.append((start_tuple, end_tuple))
                    except ValueError as e:
                        rospy.logwarn(f"Skipping malformed line: {line.strip()}. Error: {e}")
            return lines
    except IOError as e:
        rospy.logerr(f"Failed to read data from {filename}: {e}")
        return []




def calculate_lines_and_save():
    filename = "/home/neobotix/ros_workspace/src/street_robot/scripts/calculated_lines.txt"

    # Step 1: Save the robot's current position
    save_current_position(filename)

    # Step 2: Calculate the start point
    start_x, start_y = calculate_start_point()

    # Step 3: Calculate all line coordinates
    lines = calculate_lines(start_x, start_y, LINE_LENGTH, DISTANCE_BETWEEN_LINES, LINE_NUMBER, LINE_DIRECTION)

    # Step 4: Save calculated data to file
    try:
        with open(filename, 'w') as file:  # Overwrite the file
            file.write(f"Current Position: ({x_current:.2f}, {y_current:.2f})\n")
            file.write(f"Calculated Start Point: ({start_x:.2f}, {start_y:.2f})\n")
            for i, (start, end) in enumerate(lines, 1):
                file.write(f"Line {i}: Start {start}, End {end}\n")
        rospy.loginfo(f"Data saved to file: {filename}")
    except IOError as e:
        rospy.logerr(f"Failed to save data: {e}")

    # Signal comparison thread
    comparison_ready.set()
    rospy.loginfo("Line data saved. Comparison can start.")
    
   
####################################  20  the main ###################################  
def main():
    global hardware_run_pub, hardware_stop_pub

    rospy.init_node('mp400_goal_sender', anonymous=True)
    
    # Initialize publishers
    hardware_run_pub = rospy.Publisher('hardware_run', String, queue_size=10)
    hardware_stop_pub = rospy.Publisher('hardware_stop', String, queue_size=10)

    # Initialize subscribers
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, current_pose_callback)
    rospy.Subscriber('/move_base/status', GoalStatusArray, status_callback)
  #  rospy.Subscriber('/lidar_1/scan', LaserScan, laser_callback)

    rospy.Subscriber('scan', LaserScan, laser_callback)
    rospy.Subscriber('/obst_detect', String, obst_detect_callback)



    rospy.sleep(2)

    # Start comparison thread
    comparison_thread = threading.Thread(target=run_comparison, daemon=True)
    comparison_thread.start()
    

    # Perform calculations and save lines
    calculate_lines_and_save()  # Now Python knows this function exists!

    # Step 5: Move the robot to execute the calculated lines
    filename = "/home/neobotix/ros_workspace/src/street_robot/scripts/calculated_lines.txt"
    lines = read_saved_lines_safe(filename)  # Ensure the function is defined
    
    send_lines_as_goals(lines, LINE_DIRECTION)
    hardware_stop_pub.publish("stop")
    publish_mqtt(MQTT_FINISH_DRAWING, "Drawing completed successfully")
    rospy.loginfo("Finished processing all goals.")
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
