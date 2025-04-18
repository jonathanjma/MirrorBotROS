import serial
import time
import datetime
import rospy
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String
import numpy as np
from MirrorBot_head import head
from pynput import keyboard
import threading

# Serial configuration
bluetooth_port = '/dev/ttyACM0'
baud_rate = 9600
arduino = None
last_command = 'x'
control_enabled = False

# Set of currently held keys
pressed_keys = set()

# Map of valid robot control keys
navigation_keys = {'w', 'a', 's', 'd', 'q', 'e'}
mirror_keys = {'1', '2', '3', '4', '5', '6', '7', '8', '0'}

# Initialize ROS node
rospy.init_node('combined_robot_control', anonymous=True)

# Initialize MirrorBot head
robot_head = head()

# Mirror tracking filter
last_time = 0
throttle_time = 200
num_people = 0

# Action selected by key press
action = None
action_lock = threading.Lock()

# Start datetime for logging
start_datetime = datetime.datetime.now().strftime("%m-%d-%Y_%H-%M-%S")
datetime_action_logs = []

# Mirror dimensions
IN_TO_M = 39.37
MIRROR_X = 8 / IN_TO_M
MIRROR_X_RIGHT = 1.5 / IN_TO_M
MIRROR_Y = 12 / IN_TO_M
MIRROR_Y_LEFT = 14 / IN_TO_M
MIRROR_Z = 4 / IN_TO_M

# Send a command to Arduino
def send_command(cmd):
    global last_command
    if arduino and cmd != last_command:
        arduino.write(cmd.encode())
        print(f"➡️ Sent: {cmd}")
        last_command = cmd

# On key press
def on_press(key):
    global control_enabled, action
    try:
        k = key.char
        current_time = datetime.datetime.now().strftime("%m/%d/%Y %H:%M:%S")
        if control_enabled and k in navigation_keys:
            pressed_keys.add(k)
            send_command(k)
        elif k in mirror_keys:
            with action_lock:
                action = k
            print(f"\nAction {action} occurs at time {current_time}")
            datetime_action_logs.append(f"{current_time}: Action {action}")
    except AttributeError:
        if key == keyboard.Key.esc:
            print("🚪 Exiting...")
            return False
        elif key == keyboard.Key.tab:
            control_enabled = not control_enabled
            print(f"{'✅ Control Enabled' if control_enabled else '🚫 Control Disabled'}")
            if not control_enabled:
                send_command('x')  # Stop motors when turning control off

# On key release
def on_release(key):
    if not control_enabled:
        return
    try:
        k = key.char
        if k in navigation_keys and k in pressed_keys:
            pressed_keys.remove(k)
        if not any(k in navigation_keys for k in pressed_keys):
            send_command('x')
    except AttributeError:
        pass

# Callback function to process incoming PointCloud messages
def face_cloud_callback(msg):
    global last_time, num_people
    current_time = rospy.get_time() * 1000
    if current_time - last_time < throttle_time:
        return

    if len(msg.points) >= 2:
        num_people = len(msg.points)
        sorted_points = sorted(msg.points, key=lambda p: p.z)

        faceIdx = 0 if (sorted_points[0].x < sorted_points[1].x) else 1
        face_mirror = np.array([sorted_points[faceIdx].x, -sorted_points[faceIdx].y, sorted_points[faceIdx].z])  # camera has neg y axis going up

        # Calculate mirror angles for left and right mirrors
        # (Implement the angle calculations as per your original face_position_bridge_2.py logic)

# Function to execute mirror actions based on the selected action
def execute_action():
    global action
    with action_lock:
        current_action = action
        action = None
    if current_action == '1':
        look_to_lp()
    elif current_action == '2':
        look_to_rp()
    elif current_action == '3':
        connect_singlel_connectr()
    elif current_action == '4':
        connect_connectl_singler()
    elif current_action == '5':
        mirror_down()
    elif current_action == '6':
        robot_head.mirror_nod(1, 1, 6)
    elif current_action == '7':
        robot_head_pan(False, True, 2)
    elif current_action == '8':
        connect_connectl_connectr()
    elif current_action == '0':
        robot_head.homes()

# Define all the mirror movement functions here
def look_to_rp():
    robot_head.mirror_move(theta_horiz_single_rr, theta_vert_single_rr, theta_horiz_single_rl, theta_vert_single_rl)
    time.sleep(1)
    print(theta_horiz_single_r, ",", theta_vert_single_r, ",", theta_horiz_single_l, ",", theta_vert_single_l)

def look_to_lp():
    robot_head.mirror_move(theta_horiz_single_r - 1, theta_vert_single_r - 1, theta_horiz_single_l, theta_vert_single_l - 1)
    time.sleep(1)
    print(theta_horiz_single_r, ",", theta_vert_single_r, ",", theta_horiz_single_l, ",", theta_vert_single_l)

def connect_singlel_connectr():
    robot_head.mirror_move(theta_horiz_connect_r, theta_vert_connect_r, theta_horiz_single_l, theta_vert_single_l)
    print(f"Connect- Theta Horizontal: {theta_horiz_connect_r}, Theta Vertical: {theta_vert_connect_r}")

def connect_connectl_singler():
    robot_head.mirror_move(theta_horiz_single_rl + 5, theta_vert_single_rl, theta_horiz_connect_l - 8, theta_vert_connect_l)

def connect_connectl_connectr():
    robot_head.mirror_move(theta_horiz_connect_r, theta_vert_connect_r, theta_horiz_connect_l - 8, theta_vert_connect_l)

def mirror_down():
    r = 1
    move_by = 45
    robot_head.mirror_move(0, (-45) * int(r), 0, (-45) * int(r))
    time.sleep(0.8)

def robot_head_pan(r: bool, l: bool, duration: int):
    print("Starting to pan!")
    t_start = time.time()
    nod_by = 30
    while ((time.time() - t_start) < duration

 

