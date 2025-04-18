#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String
import numpy as np
import serial
import time
import datetime
from MirrorBot_head import head
from pynput import keyboard
import atexit



IN_TO_M = 39.37
MIRROR_X = 8 / IN_TO_M
# MIRROR_X_RIGHT = 4.5 / IN_TO_M
MIRROR_X_RIGHT = 1.5 / IN_TO_M
MIRROR_Y = 12 / IN_TO_M

MIRROR_Y_LEFT = 14 / IN_TO_M
#MIRROR_Y_LEFT = 10 / IN_TO_M

MIRROR_Z = 4 / IN_TO_M

class Mirror_actions():
    def __init__(self):
        print("hwaaaat")
        self.robot_head = head()
        print("party")
        rospy.init_node('face_cloud_subscriber', anonymous=True)
        print("rock")
        rospy.Subscriber('/face_detector/faces_cloud', PointCloud, self.face_cloud_callback)
        rospy.Subscriber('/gpt_action', String, self.gpt_action_callback)
        print("in")
        # mirror tracking filter
        self.last_time = 0
        self.throttle_time = 200
        self.num_people = 0

        # keyboard input
        self.action = None  # Action selected by key press
        self.listener = keyboard.Listener(on_press=self.on_press)  # Start the keyboard listener
        self.listener.start()

        # action logger
        self.start_datetime = datetime.datetime.now().time()
        self.datetime_action_logs = []

    def gpt_action_callback(self, action):
        try:
            action = int(action.data)
            if action >= 0 and action <= 7:
                self.action = str(action)
        except:
            pass

    def on_press(self, key):
        try:
            # Check key.char for printable characters
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
        except AttributeError:
            # This handles special keys (non-printable keys) if needed
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
        #     print(f"Point: x={point.x}, y={point.y}, z={point.z}")
        
        # print(f"Faces: {len(msg.points)}")

        if len(msg.points) >= 2:
            self.num_people  = len(msg.points)
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
#        self.robot_head.mirror_move(self.theta_horiz_single_r, self.theta_vert_single_r, self.theta_horiz_single_l, self.theta_vert_single_l)

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
        while not rospy.is_shutdown():             
            if self.num_people >=2 :    
                try:
                    current_time = datetime.datetime.now().time()
                    print("\nAction ", self.action, " occurs at time ", current_time)
                    self.datetime_action_logs.append(current_time, ": Action ", self.action, " executed\n")
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

                except:
                    print("waiting for points")

    @atexit.register
    # Saves all logging information to a txt file
    def on_close(self):
        filename = "datetime_action_log_" + self.start_datetime + ".txt"
        with open(filename, 'w') as f:
            for line in self.datetime_action_logs:
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
