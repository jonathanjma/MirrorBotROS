#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, Image, PointField
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Header

fx, fy = 640, 640 # in pixels
cx, cy = 640, 400 # in pixels


class RealSenseSubscriber:
    def __init__(self):
        rospy.init_node('realsense_subscriber', anonymous=True)

        # Topics for point cloud and RGB images
        self.point_cloud_topic = "/camera/depth/color/points"  # Adjust this based on your RealSense setup
        self.rgb_image_topic = "/camera/color/image_raw"       # Adjust this based on your RealSense setup

        # Subscribers
        self.point_cloud_sub = rospy.Subscriber(self.point_cloud_topic, PointCloud2, self.point_cloud_callback)
        self.rgb_image_sub = rospy.Subscriber(self.rgb_image_topic, Image, self.rgb_image_callback)
        self.points_list = []
        # CVBridge to convert ROS Image messages to OpenCV format
        self.bridge = CvBridge()
        self.pointcloud_verifier_pub = rospy.Publisher("/pointcloud_with_color", PointCloud2, queue_size=10)
 

    def point_cloud_callback(self, msg):
        """Callback function to process incoming PointCloud2 messages."""
        rospy.loginfo("Received a PointCloud2 message")
        points_list = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        self.points_list = points_list
        # Optional: Process or save the point cloud
        rospy.loginfo("The first 10 points in our point cloud")
        for i in range(0,100000,10000):
            current_point = points_list[i]
            rospy.loginfo("Point cloud " + str(i) + " has depth " + str(current_point[2]))
        rospy.loginfo("Point cloud has %d points", len(points_list))
        

    def rgb_image_callback(self, msg):
        """Callback function to process incoming RGB image messages."""
        rospy.loginfo("Received an RGB image")
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.pixels = cv_image
            rospy.loginfo("Shape of the self.pixels variable "+ str(self.pixels.shape))
            # Display the image (optional)
           
            if (len(self.points_list) != 0):
                points_list = self.points_list
                #rgb_made_from_point_cloud = self.pointcloud_to_image(points_list,fx,fy,cx,cy)
                #rospy.loginfo("RGB Image made from Point cloud has %d points", len(rgb_made_from_point_cloud))
                #rospy.loginfo("First pixel is " + str(rgb_made_from_point_cloud[0]))
                depth_list = []
                #for point in points_list:
                #    depth_list.append(point[2])
                #modified_image = self.draw_gradient_pixels_on_ros_image(cv_image,rgb_made_from_point_cloud, depth_list)
                #cv2.imshow("Modified RGB Image", modified_image)
            cv2.imshow("RGB Image", cv_image)
            cv2.waitKey(1)  # Add delay to update OpenCV window

        except Exception as e:
            rospy.logerr("Failed to convert image: %s", str(e))
    
    def pointcloud_to_image(self, points, fx, fy, cx, cy):
        """
        Converts a list of 3D points (x, y, z) from a point cloud to 2D RGB image pixels.

        Args:
            points (list of tuples): List of (x, y, z) points from the point cloud.
            fx (float): Focal length in x direction.
            fy (float): Focal length in y direction.
            cx (float): Principal point in x direction.
            cy (float): Principal point in y direction.

        Returns:
            list of tuples: List of (u, v) pixel coordinates in the RGB image.
        """
        pixels = []
        for x, y, z in points:
            if z > 0:  # Ignore points behind the camera
                u = int((x / z) * fx + cx)
                v = int((y / z) * fy + cy)
                pixels.append((u, v))
        return pixels
    
    def image_to_pointcloud(self, pixels, depth_values, fx, fy, cx, cy):
        """
        Maps 2D pixel coordinates with depth values back to 3D points in the camera frame.

        Args:
            pixel_coords (list of tuples): List of (u, v) pixel coordinates from the image.
            depth_values (list of floats): List of depth values corresponding to each pixel.

        Returns:
            numpy.ndarray: A (N, 3) array of 3D points (X, Y, Z) in the camera frame.
        """
        #global fx, fy, cx, cy  # Use global intrinsic parameters

        # Ensure the input lists are of the same length
        if len(pixels) != len(depth_values):
            raise ValueError("Pixel coordinates and depth values must have the same length.")

        # Initialize a list to store 3D points
        pointcloud = []

        for (u, v), depth in zip(pixels, depth_values):
            if depth > 0:  # Ignore invalid depth values
                # Back-project the pixel to 3D using the pinhole camera model
                x = (u - cx) * depth / fx
                y = (v - cy) * depth / fy
                z = depth
                pointcloud.append([x, y, z])

        return pointcloud



    # Allows you to draw colored pixels (based on pixel depth) on an image (useful for knowing what pixels of the rgb camera I obtained from the PointCloud)
    def draw_gradient_pixels_on_ros_image(self, cv_image, pixel_list, depth_list):
        """
        Draws pixels with a gradient color based on depth (ROYGBIV) on an image received from a ROS topic.

        Args:
            cv_image (numpy.ndarray): The image converted from ROS Image message.
            pixel_list (list of tuples): List of (u, v) pixel coordinates.
            depth_list (list of floats): Corresponding depth values for each pixel.

        Returns:
            numpy.ndarray: The modified image with a depth-based gradient applied to pixels.
        """
        # Ensure the depth_list matches the pixel_list
        if len(pixel_list) != len(depth_list):
            raise ValueError("Pixel list and depth list must have the same length.")

        # ROYGBIV color mapping (BGR format) for gradient interpolation
        colors = [
            (0, 0, 255),    # Red
            (0, 127, 255),  # Orange
            (0, 255, 255),  # Yellow
            (0, 255, 0),    # Green
            (255, 255, 0),  # Blue
            (255, 0, 255),  # Indigo
            (128, 0, 128),  # Violet
        ]

        # Create a copy of the original image
        cv_image_copy = cv_image.copy()

        # Normalize depth values to range [0, 1] for gradient computation
        min_depth, max_depth = min(depth_list), max(depth_list)
        depth_range = max_depth - min_depth if max_depth > min_depth else 1

        for (u, v), depth in zip(pixel_list, depth_list):
            # Ensure the pixel is within the image bounds
            if 0 <= v < cv_image_copy.shape[0] and 0 <= u < cv_image_copy.shape[1]:
                # Normalize depth to [0, 1]
                normalized_depth = (depth - min_depth) / depth_range

                # Interpolate color based on normalized depth
                color_index = normalized_depth * (len(colors) - 1)
                low_idx = int(color_index)  # Lower bound index
                high_idx = min(low_idx + 1, len(colors) - 1)  # Upper bound index
                alpha = color_index - low_idx  # Interpolation factor

                # Interpolate between the two nearest colors
                color_low = np.array(colors[low_idx], dtype=float)
                color_high = np.array(colors[high_idx], dtype=float)
                interpolated_color = (1 - alpha) * color_low + alpha * color_high

                # Apply the color to the pixel
                cv_image_copy[v, u] = interpolated_color.astype(np.uint8)

        return cv_image_copy

    def map_image_to_pointcloud_with_color(pixel_coords, depth_values, fx, fy, cx, cy, image):
        """
        Maps 2D image pixels to a 3D point cloud and overlays pixel colors onto the points.

        Args:
            pixel_coords (list of tuples): List of (u, v) pixel coordinates from the image.
            depth_values (list of floats): List of depth values corresponding to each pixel.
            fx (float): Focal length in the x-direction.
            fy (float): Focal length in the y-direction.
            cx (float): Principal point x-coordinate.
            cy (float): Principal point y-coordinate.
            image (numpy.ndarray): The 2D RGB image as a NumPy array.

        Returns:
            list: A list of 3D points with color information as tuples (x, y, z, r, g, b).
        """
        # Ensure the input lists are of the same length
        if len(pixel_coords) != len(depth_values):
            raise ValueError("Pixel coordinates and depth values must have the same length.")

        # Initialize the list to store points with colors
        pointcloud_with_color = []

        for (u, v), depth in zip(pixel_coords, depth_values):
            if depth > 0 and not np.isnan(depth):  # Ignore invalid depth values
                # Back-project the pixel to 3D using the pinhole camera model
                x = (u - cx) * depth / fx
                y = (v - cy) * depth / fy
                z = depth

                # Get the color of the corresponding pixel
                if 0 <= v < image.shape[0] and 0 <= u < image.shape[1]:
                    r, g, b = image[v, u]  # Pixel color (RGB)
                    pointcloud_with_color.append((x, y, z, r, g, b))

        return pointcloud_with_color

    def publish_pointcloud(pointcloud_with_color, frame_id="camera_frame"):
        """
        Publishes a point cloud to a ROS topic.

        Args:
            pointcloud_with_color (list): List of 3D points with color (x, y, z, r, g, b).
            frame_id (str): The frame ID for the point cloud.
        """
        # Define the header
        header = Header()
        header.frame_id = frame_id
        header.stamp = rospy.Time.now()

        # Define the fields for PointCloud2
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1)
        ]

        # Convert the point cloud to a generator for PointCloud2
        def pointcloud_generator():
            for x, y, z, r, g, b in pointcloud_with_color:
                # Pack RGB into a single 32-bit integer
                rgb = (int(r) << 16) | (int(g) << 8) | int(b)
                yield [x, y, z, rgb]

        # Create the PointCloud2 message
        pc2_msg = pc2.create_cloud(header, fields, pointcloud_generator())

        # Publish the message
        pub.publish(pc2_msg)



    def run(self):
        """Keep the node running."""
        rospy.spin()

if __name__ == '__main__':
    try:
        realsense_subscriber = RealSenseSubscriber()
        realsense_subscriber.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
