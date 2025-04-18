#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String
import numpy as np
import serial
import time
from MirrorBot_head import head
from pynput import keyboard
import datetime
import atexit
import sys
import traceback


"""
REMEMBER!!!

The wheel servos and the mirror servos are controlled by different arduinos.
"""

IN_TO_M = 39.37
MIRROR_X = 8 / IN_TO_M
# MIRROR_X_RIGHT = 4.5 / IN_TO_M
MIRROR_X_RIGHT = 1.5 / IN_TO_M
MIRROR_Y = 12 / IN_TO_M

MIRROR_Y_LEFT = 14 / IN_TO_M
#MIRROR_Y_LEFT = 10 / IN_TO_M

MIRROR_Z = 4 / IN_TO_M

start_datetime = datetime.datetime.now().strftime("%m-%d-%Y_%H-%M-%S")
datetime_action_logs = []

# Set of currently held keys, for wheel movement
pressed_keys_wheels = set()

# Map of valid robot control keys, for wheel movement
movement_keys_wheels = {'w', 'a', 's', 'd', 'q', 'e'}

# these are serial connection details only for the Arduino of the Wheels
bluetooth_port_arduino_wheels = '/dev/ttyACM0'
baud_rate_arduino_wheels = 9600
arduino_wheels = None
last_command_wheels = 'x'
control_enabled_wheels = False

class Mirror_actions():
    def __init__(self):
        print("hwaaaat")
        self.robot_head = head()
        print("party")
        rospy.init_node('face_cloud_subscriber', anonymous=True)
        print("rock")
        rospy.Subscriber('/face_detector/faces_cloud', PointCloud, self.face_cloud_callback)
        rospy.Subscriber('/gpt_action', String, self.gpt_action_callback)
        #rospy.Subscriber('/mirror_cmd', String, self.mirror_cmd_callback)
        print("in")
        # mirror tracking filter
        self.last_time = 0
        self.throttle_time = 200
        self.num_people = 0

        # keyboard input
        self.action = None # Action selected by key press
        self.listener = keyboard.Listener(on_press=self.on_press,on_release=self.on_release) # Start the keyboard listener
        self.listener.start()

    def gpt_action_callback(self, action):
        try:
            action = int(action.data)
            if action >= 0 and action <= 7:
                self.action = str(action)
        except:
            pass

        # Send a command to Arduino
    def send_command(self, cmd):
        global last_command_wheels
        if arduino_wheels and cmd != last_command_wheels:
            arduino_wheels.write(cmd.encode())
            print(f"➡️ Sent: {cmd}")
            last_command_wheels = cmd


    def on_press(self, key):
        global control_enabled_wheels
        # Check key.char for printable characters
        current_time = datetime.datetime.now().strftime("%m/%d/%Y %H:%M:%S")
        try:
            # mirror mode keys
            print("???", key.char)
            print("control enabled", control_enabled_wheels)
            print("movement keys wheels", movement_keys_wheels)
            if key.char == '1':
                self.action = '1'
            elif key.char == '2':
                self.action = '2'
            elif key.char == '3':
                self.action = '3'
            elif key.char == '4':
                self.action = '4'
            elif key.char == '5':
                self.action = '5'
            elif key.char == '6':
                self.action = "6"
            elif key.char == '7':
                self.action = "7"
            elif key.char == "8":
                self.action = '8'
            elif key.char == "0":
                self.action = '0'
                print("0 ing!")

            # wheel mode keys
            elif (key.char in movement_keys_wheels and control_enabled_wheels):
                pressed_keys_wheels.add(key.char) # only add key.char if it is one
                                                  # of the movement_key_wheel characters
                self.send_command(key.char) # send the pressed key command to the
                                            # arduino
                self.action = key.char
                print("wheels char", key.char)

            if self.action is not None:
                print("\nAction ", self.action, " occurs at time ", current_time)
                datetime_action_logs.append(current_time + ": Action " + str(self.action))

        except AttributeError:
            print("throw an error please")
            #print("key char", key.char)
            if key == keyboard.Key.esc:
                print("🚪 Exiting...")
                return False
            elif key == keyboard.Key.tab:
                control_enabled_wheels = not control_enabled_wheels
                print(f"{'✅ Control Enabled' if control_enabled_wheels else '🚫 Control Disabled'}")
                if not control_enabled_wheels:
                    self.send_command('x') # Stop motors when turning control off
        #except:
         #   print("key", key)
        #    print("how did we get here?")

    def on_release(self,key):
        if not control_enabled_wheels:
            return
        try:
            k = key.char
            if k in movement_keys_wheels and k in pressed_keys_wheels:
                pressed_keys_wheels.remove(k)
            if not any(k in movement_keys_wheels for k in pressed_keys_wheels):
                self.send_command('x')
        except AttributeError:
            pass


    def face_cloud_callback(self, msg):
        """
        Callback function to process incoming PointCloud messages.
        """
        # throttle callback to prevent Arduino serial from getting overwhelmed
        current_time = rospy.get_time()*1000
        if current_time - self.last_time < self.throttle_time:
            return

        # rospy.loginfo("Received a point cloud message on /face_detector/face_cloud")
        # Iterate through the points in the PointCloud
        # Each point is an instance of an individual
        # for point in msg.points:
        # print(f"Point: x={point.x}, y={point.y}, z={point.z}")
        # print(f"Faces: {len(msg.points)}")

        if len(msg.points) >= 2:
            self.num_people = len(msg.points)
            sorted_points = sorted(msg.points, key=lambda p: p.z)

            faceIdx = 0 if (sorted_points[0].x < sorted_points[1].x) else 1
            face_mirror = np.array([sorted_points[faceIdx].x, -sorted_points[faceIdx].y, sorted_points[faceIdx].z]) # camera has neg y axis going up
            # --------------------------------------------------------------------------------------------------------------
            ## left person self reflection

            # left mirror self-reflection
            self.theta_horiz_single_l = int(np.degrees(np.arctan2(-MIRROR_X - face_mirror[0], face_mirror[2] - MIRROR_Z)))
            self.theta_vert_single_l = int(np.degrees(np.arctan2(face_mirror[1] - (MIRROR_Y_LEFT), face_mirror[2] - MIRROR_Z)))

            # right mirror self-reflection
            self.theta_horiz_single_r = int(np.degrees(np.arctan2(MIRROR_X_RIGHT - face_mirror[0], face_mirror[2] - MIRROR_Z)))
            self.theta_vert_single_r = int(np.degrees(np.arctan2(face_mirror[1] - (MIRROR_Y), face_mirror[2] - MIRROR_Z)))
            # --------------------------------------------------------------------------------------------------------------
            ## right person self reflection

            faceIdx1 = 0 if (sorted_points[0].x > sorted_points[1].x) else 1
            face_mirror1 = np.array([sorted_points[faceIdx1].x, -sorted_points[faceIdx1].y, sorted_points[faceIdx1].z]) # camera has neg y axis going up

            # left mirror reflection of the right person
            self.theta_horiz_single_rl = int(np.degrees(np.arctan2( ( -MIRROR_X - face_mirror1[0]), (face_mirror1[2] - MIRROR_Z))))
            self.theta_vert_single_rl = int(np.degrees(np.arctan2(face_mirror1[1] - MIRROR_Y_LEFT, face_mirror1[2] - MIRROR_Z)))

            # right mirror reflection of the right person
            self.theta_horiz_single_rr = int(np.degrees(np.arctan2( (MIRROR_X_RIGHT - face_mirror1[0]), (face_mirror1[2] - MIRROR_Z))))
            self.theta_vert_single_rr = int(np.degrees(np.arctan2(face_mirror1[1] - MIRROR_Y, face_mirror1[2] - MIRROR_Z)))
            # --------------------------------------------------------------------------------------------------------------
            ## right mirror mutural reflection, left mirror left person reflection

            face1 = np.array([sorted_points[0].x, -sorted_points[0].y, sorted_points[0].z]) # camera has neg y axis going up
            face2 = np.array([sorted_points[1].x, -sorted_points[1].y, sorted_points[1].z])
            midpoint = (face1 + face2) / 2
            print(midpoint)
            self.theta_horiz_connect_r = int(np.degrees(np.arctan2(MIRROR_X_RIGHT - midpoint[0], midpoint[2] - MIRROR_Z)))
            self.theta_vert_connect_r = int(np.degrees(np.arctan2(midpoint[1] - MIRROR_Y, midpoint[2] - MIRROR_Z)))
            # --------------------------------------------------------------------------------------------------------------
            ## left mirror mutural reflection, right mirror right person reflection

            self.theta_horiz_connect_l = int(np.degrees(np.arctan2((-MIRROR_X_RIGHT - midpoint[0]), (midpoint[2] - MIRROR_Z))))
            self.theta_vert_connect_l = int(np.degrees(np.arctan2((midpoint[1] - MIRROR_Y_LEFT), (midpoint[2] - MIRROR_Z))))

    def look_to_rp(self):
        # self.robot_head.mirror_move(self.theta_horiz_single_rr-10, self.theta_vert_single_rr-8, self.theta_horiz_single_rl+30, self.theta_vert_single_rl-8)
        # self.robot_head.mirror_move(self.theta_horiz_single_r-20, self.theta_vert_single_r-9, self.theta_horiz_single_l-20, self.theta_vert_single_l-10)
        self.robot_head.mirror_move(self.theta_horiz_single_rr, self.theta_vert_single_rr, self.theta_horiz_single_rl, self.theta_vert_single_rl)
        time.sleep(1)
        print(self.theta_horiz_single_r, "," , self.theta_vert_single_r, ",", self.theta_horiz_single_l, ",", self.theta_vert_single_l)

    def look_to_lp(self):
        self.robot_head.mirror_move(self.theta_horiz_single_r-1, self.theta_vert_single_r-1, self.theta_horiz_single_l, self.theta_vert_single_l-1)
        time.sleep(1)
        print(self.theta_horiz_single_r, "," , self.theta_vert_single_r, ",", self.theta_horiz_single_l, ",", self.theta_vert_single_l)

    def connect_singlel_connectr(self):
        self.robot_head.mirror_move(self.theta_horiz_connect_r, self.theta_vert_connect_r, self.theta_horiz_single_l, self.theta_vert_single_l)
        # self.robot_head.mirror_move(self.theta_horiz_single_r, self.theta_vert_single_r, self.theta_horiz_single_l, self.theta_vert_single_l)

        print(f"Connect- Theta Horizontal: {self.theta_horiz_connect_r}, Theta Vertical: {self.theta_vert_connect_r}")

    def connect_connectl_singler(self):
        self.robot_head.mirror_move(self.theta_horiz_single_rl+5, self.theta_vert_single_rl, self.theta_horiz_connect_l-8, self.theta_vert_connect_l)
        # print(f"Connect- Theta Horizontal: {self.theta_horiz_connect_r}, Theta Vertical: {self.theta_vert_connect_r}")

    def connect_connectl_connectr(self):
        self.robot_head.mirror_move(self.theta_horiz_connect_r, self.theta_vert_connect_r, self.theta_horiz_connect_l-8, self.theta_vert_connect_l)

    def mirror_down(self):
        r = 1
        move_by = 45
        self.robot_head.mirror_move(0, (-45) * int(r), 0, (-45) * int(r))
        time.sleep(0.8)

    def robot_head_pan(self, r:bool, l:bool, duration:int):
        print("Starting to pan!")
        t_start = time.time()
        print("Got the time!")
        nod_by = 30
        while ((time.time()-t_start) < duration):
            print("Panning my head")
            self.robot_head.mirror_move((self.robot_head.home_pos + nod_by) * int(r), 0, (self.robot_head.home_pos + nod_by) * int(l), 0)
            time.sleep(0.8)
            self.robot_head.mirror_move((self.robot_head.home_pos - nod_by) * int(r), 0, (self.robot_head.home_pos - nod_by) * int(l), 0)
            time.sleep(0.8)
        self.robot_head.homes()

    def run(self):
        """
        If the wheels don't work, the program will just stop haha.
        Please be mindful of this. If you would like different behavior, please
        let me know.
        """
        global arduino_wheels
        try:
            arduino_wheels = serial.Serial(bluetooth_port_arduino_wheels, baud_rate_arduino_wheels, timeout=1)
            print(f"✅ Connected to Arduino on {bluetooth_port_arduino_wheels}")
        except serial.SerialException:
            print("❌ Failed to connect to Arduino.")
            return # this line is the reason why the program will stop if the
            # wheels don't work.
        while not rospy.is_shutdown():
            if self.num_people >=2 :
                try:
                    if self.action == '1':
                        self.look_to_lp()
                    elif self.action == "2":
                        self.look_to_rp()
                    elif self.action == "3":
                        self.connect_singlel_connectr()
                    elif self.action == "4":
                        self.connect_connectl_singler()
                    elif self.action == "5":
                        self.mirror_down()
                    elif self.action == "6":
                        self.robot_head.mirror_nod(1,1,6)
                        self.action == None
                    elif self.action == "7":
                        self.robot_head_pan(False, True, 2)
                    elif self.action == "8":
                        self.connect_connectl_connectr()
                    elif self.action == "0":
                        self.robot_head.homes()
                        # Be aware that I did not include the print statement that
                        # prints out responses, from the Arduino to the computer.
                except KeyboardInterrupt:
                    print("⛔ Interrupted.")
                except:
                    print("waiting for points")
                #finally:
                #    arduino_wheels.close()
                #    print("🔌 Serial connection closed.")
                #    self.listener.stop()

@atexit.register
# Saves all logging information to a txt file
def on_close():
    filename = "datetime_action_log_" + str(start_datetime) + ".txt"
    with open(filename, 'w') as f:
        for line in datetime_action_logs:
            f.write(f"{line}\n")
    print('Logs saved to ', filename) #save my data

if __name__ == '__main__':
    print("what")
    try:
        print("hey!")
        task = Mirror_actions()
        print("task initialization works!")
        task.run()
        print("task run works!")
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception. Node shutting down.")