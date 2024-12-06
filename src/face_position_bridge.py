#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud
import numpy as np
import serial

IN_TO_M = 39.37
MIRROR_X_LEFT = 5.5 / IN_TO_M
MIRROR_X_RIGHT = 6 / IN_TO_M
MIRROR_Y = 12 / IN_TO_M
MIRROR_Z = 6 / IN_TO_M

arduino = serial.Serial(port='/dev/ttyACM1')

def face_cloud_callback(msg):
    """
    Callback function to process incoming PointCloud messages.
    """
    # rospy.loginfo("Received a point cloud message on /face_detector/face_cloud")
    
    # Iterate through the points in the PointCloud
    # Each point is an instance of an individual
    # for point in msg.points:
    #     print(f"Point: x={point.x}, y={point.y}, z={point.z}")
    
    rospy.loginfo(f"Faces: {len(msg.points)}")

    if len(msg.points) == 2:

        faceIdx = 0 if (msg.points[0].y < msg.points[1].y) else 1
        face_mirror = np.array([msg.points[faceIdx].x, -msg.points[faceIdx].y, msg.points[faceIdx].z]) # camera has neg y axis going up
        theta_horiz_single = int(np.degrees(np.arctan2(MIRROR_X_LEFT - face_mirror[0], face_mirror[2] - MIRROR_Z)))
        theta_vert_single = int(np.degrees(np.arctan2(face_mirror[1] - MIRROR_Y, face_mirror[2] - MIRROR_Z)))
        arduino.write(("ML_X " + str(theta_horiz_single) + "\n").encode())
        arduino.write(("ML_Y " + str(theta_vert_single) + "\n").encode())
        print(f"Single- Theta Horizontal: {theta_horiz_single}, Theta Vertical: {theta_vert_single}")
        
        # --------------------------------------------------------------------------------------------------------------
        face1 = np.array([msg.points[0].x, -msg.points[0].y, msg.points[0].z]) # camera has neg y axis going up
        face2 = np.array([msg.points[1].x, -msg.points[1].y, msg.points[1].z])
        midpoint = (face1 + face2) / 2
        print(midpoint)

        theta_horiz_connect = int(np.degrees(np.arctan2(MIRROR_X_RIGHT - midpoint[0], midpoint[2] - MIRROR_Z)))
        theta_vert_connect = int(np.degrees(np.arctan2(midpoint[1] - MIRROR_Y, midpoint[2] - MIRROR_Z)))
        print(f"Connect- Theta Horizontal: {theta_horiz_connect}, Theta Vertical: {theta_vert_connect}")
        arduino.write(("MR_X " + str(theta_horiz_connect) + "\n").encode())
        arduino.write(("MR_Y " + str(theta_vert_connect) + "\n").encode())

def face_cloud_subscriber():
    """
    Initializes the ROS node and subscribes to the /faces_detector/face_cloud topic.
    """
    rospy.init_node('face_cloud_subscriber', anonymous=True)
    rospy.Subscriber('/face_detector/faces_cloud', PointCloud, face_cloud_callback)

    print("Subscribed to /face_detector/faces_cloud")
    rospy.spin()

if __name__ == '__main__':
    try:
        face_cloud_subscriber()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception. Node shutting down.")
