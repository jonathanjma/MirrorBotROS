MirrorBot.py 


Packages required: realsense2_sdk, wg-perception/people
Do not execute in conda env !

##### MirrorBot Arduino Tunning #####
To change the velocity of the base:
	Run arduino-ide_2.3.4_Linux_64bit.AppImage at /home
	Open Movement_Controller, select the board
	Change "linear_speed" at line 6 for linear velocity (max 200, min 0, recommend 70-170)
	Change "speed" at line 7 for angular velocity (max 200, min 0, recommend 70-90)
	Upload the code to Arduino

##### MirrorBot Face Detection & Mirror Manipulation #####

To run code, run each command in a seperate terminal (make sure to run "conda deactivate")
	roslaunch realsense2_camera rs_camera.launch align_depth:=true enable_pointcloud:=true
	roslaunch face_detector face_detector.rs_rgbd.launch
	rosrun rviz rviz                                        (optional, but make sure to load /home/yyt/catkin_ws/src/mirror_bot/mirror_bot_rviz.rviz)
	python3 face_position_bridge.py                         (run in /home/yyt/catkin_ws/src/mirror_bot/src, might need to change Arduino port as needed, ACM0 or ACM1)

##### MirrorBot Play sound #####
To play a sound
	run sound_play.py in vscode (run conda deactivate first)

##### MirrorBot Record Video #####
To record video using Realsense + ROS:
	Make sure realsense node is up (if not: roslaunch realsense2_camera rs_camera.launch align_depth:=true enable_pointcloud:=true)
	In terminal: rosrun mirror_bot realsense_recorder.py
	This will record video to /home folder

