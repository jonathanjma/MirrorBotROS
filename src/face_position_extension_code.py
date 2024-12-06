#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud

def face_cloud_callback(msg):
    """
    Callback function to process incoming PointCloud messages.
    """
    rospy.loginfo("Received a point cloud message on /face_detector/face_cloud")
    
    # Iterate through the points in the PointCloud
    # Each point is an instance of an individual
    for point in msg.points:
        print(f"Point: x={point.x}, y={point.y}, z={point.z}")
    
    rospy.loginfo(f"Total points in the cloud: {len(msg.points)}")

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