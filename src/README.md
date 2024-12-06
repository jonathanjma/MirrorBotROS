MirrorBot.py manages all actuation

2. Face positions estimation:
	a) additional package needed: realsense2_sdk, wg-perception /people
	b) code to run:
	roslaunch realsense2_camera rs_camera.launch align_depth:=true enable_pointcloud:=true
	roslaunch face_detector face_detector.rs_rgbd.launch
	rosrun rviz rviz # optional for visualization
	c) example code to subscript to detection results:
	face_position_bridge.py # see comments for more info
	
