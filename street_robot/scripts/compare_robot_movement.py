#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import os
from colorama import Fore, Style
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty

import time
import threading  # Add this import
import paho.mqtt.client as paho
from actionlib_msgs.msg import GoalStatusArray, GoalID
from threading import Lock


position_lock = threading.Lock()

def get_current_position():
    with position_lock:
        return x_current, y_current

##########################################33


robot_state = "turning"  # "straight" للحركة الطولية، "turning" للالتفاف
previous_state = None  # الحالة السابقة
previous_obstacles = None  # قائمة العقبات السابقة


# Global variables
last_no_obstacle_time = 0  # Timestamp of the last "No obstacle detected" print
last_obstacle_time = 0  # Timestamp of the last "Obstacle detected" print
obstacle_print_interval = 15  # Minimum interval (in seconds) between prints (4 times per minute)

def scan_callback(scan_msg):
    """Process laser scan data and publish obstacle status."""
    global robot_state, previous_state, last_no_obstacle_time, last_obstacle_time

    # Extract angle and range data
    angle_min = scan_msg.angle_min
    angle_max = scan_msg.angle_max
    angle_increment = scan_msg.angle_increment
    ranges = scan_msg.ranges

    # Validate ranges
    if not ranges or all(math.isinf(r) for r in ranges):
        return

    # Define front angles (clamped to valid ranges)
    front_angle_min = max(angle_min, -0.09072665)  # -6.14 degrees in radians
    front_angle_max = min(angle_max, 0.09072665)   # +6.14 degrees in radians

    # Calculate indices
    index_min = max(0, int((front_angle_min - angle_min) / angle_increment))
    index_max = min(len(ranges), int((front_angle_max - angle_min) / angle_increment))

    # Filter valid front range values
    front_ranges = [r for r in ranges[index_min:index_max] if not math.isinf(r) and not math.isnan(r)]
    if not front_ranges:
        return

    # Detect obstacles within 3 meters
    obstacles = [r for r in front_ranges if r < 1.4]

    # Get the current time
    current_time = time.time()

    # Publish obstacle status
    if obstacles:
        if current_time - last_obstacle_time >= obstacle_print_interval:
            rospy.loginfo("\033[93mObstacle detected: Publishing 'yes'\033[0m")  # Yellow text for visibility
            last_obstacle_time = current_time
        obst_detect_pub.publish("yes")
    else:
        if current_time - last_no_obstacle_time >= obstacle_print_interval:
            rospy.loginfo("\033[92mNo obstacle detected: Publishing 'no'\033[0m")  # Green text for visibility
            last_no_obstacle_time = current_time
        obst_detect_pub.publish("no")

    # Handle turning state
    if robot_state == "turning" and robot_state != previous_state:
        rospy.loginfo("Robot is turning - ignoring obstacles.")

    previous_state = robot_state

   
def laser_control(filename):
    global x_current, y_current

    # Initialize the ROS publisher
    command_pub = rospy.Publisher('/robot_command', String, queue_size=10)

    # Read the saved lines from the file
    calculated_lines = read_saved_lines_safe(filename)
    if not calculated_lines:
        rospy.logerr("No valid data found in the file. Exiting laser control.")
        return

    rospy.loginfo("Starting laser control...")

    for i, (start, end) in enumerate(calculated_lines, 1):
        try:
            # Define laser points for odd and even lines
            if i % 2 != 0:  # Odd line

                laser_run_point = (start[0] + 0.5, start[1])
                
                laser_stop_point = (start[0] + 2.5, start[1])
            else:  # Even line
                laser_run_point = (start[0] - 0.5, start[1])
                laser_stop_point = (start[0] - 2.5, start[1])

            # Log the laser points for the current line
            rospy.loginfo(f"Line {i}: Laser run at {laser_run_point}, Laser stop at {laser_stop_point}")

            # Check for laser_run_point with timeout
            rospy.loginfo(f"Waiting for robot to reach laser_run_point: {laser_run_point}")
            start_time = time.time()
            while not rospy.is_shutdown():
                result = compare_positions_with_colors(laser_run_point, (x_current, y_current), tolerance)
                if result == "Matched":
                    rospy.loginfo(f"\033[38;5;214mLaser activated at {laser_run_point}. Publishing 'straight'.\033[0m")
                    command_pub.publish("straight")

                    rospy.sleep(0.1)
                    break
                if time.time() - start_time > 10:  # Timeout after 10 seconds
                    rospy.logwarn(f"Timeout waiting for laser_run_point: {laser_run_point}")
                    break
                rospy.sleep(0.1)

            # Check for laser_stop_point with timeout
            rospy.loginfo(f"Waiting for robot to reach laser_stop_point: {laser_stop_point}")
            start_time = time.time()
            while not rospy.is_shutdown():
                result = compare_positions_with_colors(laser_stop_point, (x_current, y_current), tolerance)
                if result == "Matched":
                    rospy.loginfo(f"\033[38;5;214mLaser deactivated at {laser_stop_point}. Publishing 'turning'.\033[0m")
                    command_pub.publish("turning")

                    rospy.sleep(0.1)
                    break
                if time.time() - start_time > 10:  # Timeout after 10 seconds
                    rospy.logwarn(f"Timeout waiting for laser_stop_point: {laser_stop_point}")
                    break
                rospy.sleep(0.1)

        except Exception as e:
            rospy.logerr(f"Error processing Line {i}: {e}")
            continue

    rospy.loginfo("Laser control completed for all lines.")




#############################################


# Rate-limiting variables
last_close_print_time = 0
last_different_print_time = 0
rate_limit_interval = 12  # Interval in seconds (5 messages per minute)

# Global variables
x_current = 0.0
y_current = 0.0
tolerance = 0.3  # Allowable deviation in meters

############################## 1 #######################################
def current_pose_callback(data):
    global x_current, y_current
    x_current = data.pose.pose.position.x
    y_current = data.pose.pose.position.y
   # rospy.loginfo_throttle(1, f"Current Position: (x: {x_current}, y: {y_current})")

############################## 2 #######################################
rate_lock = threading.Lock()


def compare_positions_with_colors(calculated, actual, tolerance=0.1):
    global last_close_print_time, last_different_print_time

    distance = math.sqrt((calculated[0] - actual[0])**2 + (calculated[1] - actual[1])**2)
    current_time = time.time()

    with rate_lock:
        if distance <= tolerance:
            rospy.loginfo(f"{Fore.GREEN}Matched: Calculated: {calculated}, Actual: {actual}{Style.RESET_ALL}")
            return "Matched"
        elif distance <= tolerance * 5:
            if current_time - last_close_print_time >= rate_limit_interval:
                rospy.loginfo(f"{Fore.YELLOW}Close: Calculated: {calculated}, Actual: {actual}{Style.RESET_ALL}")
                last_close_print_time = current_time
            return "Close"
        else:
            if current_time - last_different_print_time >= rate_limit_interval:
                rospy.loginfo(f"{Fore.RED}Different: Calculated: {calculated}, Actual: {actual}{Style.RESET_ALL}")
                last_different_print_time = current_time
            return "Different"

last_print_time = 0  # Keeps track of the last time a status was printed
rate_limit_interval = 12  # Rate limit interval in seconds
print_count = 0  # Counter for printed messages

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
            #    rospy.loginfo(f"Active Goal ID detected: {current_goal_id}")
                last_print_time = current_time
                print_count += 1


############################## 3 #######################################
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





def hardware_stop_callback(data):
    """
    Callback for the 'hardware_stop' topic.
    """
    rospy.logwarn(f"Hardware Stop Command Received: {data.data}")

############################## 4 #######################################

def main():
    global obst_detect_pub

    rospy.init_node('compare_robot_movement', anonymous=True)
   # rospy.init_node('obstacle_publisher', anonymous=True)

    hardware_stop_pub = rospy.Publisher('hardware_stop', String, queue_size=10)



    # Subscribers for pose and laser scans
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, current_pose_callback)
    rospy.Subscriber('/move_base/status', GoalStatusArray, status_callback)
    rospy.Subscriber('/lidar_1/scan_filtered', LaserScan, scan_callback)
    rospy.Subscriber('scan', LaserScan, scan_callback)
    
    
    
    obst_detect_pub = rospy.Publisher('/obst_detect', String, queue_size=10)

   
    filename = "/home/neobotix/ros_workspace/src/street_robot/scripts/calculated_lines.txt"
    rospy.loginfo("Starting laser control thread...")
    laser_control_thread = threading.Thread(target=laser_control, args=(filename,), daemon=True)
    laser_control_thread.start()


    rospy.sleep(2)


    calculated_lines = read_saved_lines_safe(filename)
    if not calculated_lines:
        rospy.logerr("No valid lines found in the file. Exiting.")
        return

    rate = rospy.Rate(2)  # 2 Hz, check every 0.5 seconds
    rospy.loginfo("Starting comparison of robot positions...")

    for i, (start, end) in enumerate(calculated_lines, 1):
        rospy.loginfo(f"Comparing Line {i}: Start {start}, End {end}")

        # Compare the start point
        while not rospy.is_shutdown():
            start_result = compare_positions_with_colors(start, (x_current, y_current), tolerance)
            if start_result == "Matched":
                rospy.loginfo(f"Matched start point for Line {i}: {start}")
                break
            rate.sleep()

        # Compare the end point
        while not rospy.is_shutdown():
            end_result = compare_positions_with_colors(end, (x_current, y_current), tolerance)
            if end_result == "Matched":
                hardware_stop_pub.publish("stop")  # Publish "stop" to the hardware_stop topic
                rospy.loginfo(f"Matched end point for Line {i}: {end}")
                break
            rate.sleep()

    rospy.loginfo("Comparison completed for all lines.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
