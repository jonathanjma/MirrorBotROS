MirrorBot.py 


Packages required: realsense2_sdk, wg-perception/people
Do not execute in conda env !

##### MirrorBot Arduino Tunning #####
To change the velocity of the base:
	Run arduino-ide_2.3.4_Linux_64bit.AppImage at /home
	Open Movement_Controller, select the board
	Change "linear_speed" at line 6 for linear velocity (max 200, min 0, recommend 70-170)
	Change "speed" at line 7 for angular velocity (max 200, m3444in 0, recommend 70-90)
	Upload the code to Arduino

##### MirrorBot Face Detection & Mirror Manipulation #####

To run code, run each command in a seperate terminal (make sure to run "conda deactivate")
	roslaunch realsense2_camera rs_camera.launch align_depth:=true enable_pointcloud:=true
	
	source ~/catkin_ws/devel/setup.bash
	roslaunch face_detector face_detector.rs_rgbd.launch
	
	rosrun rviz rviz (optional, but make sure to load /home/mirrorbot/catkin_ws/src/mirror_bot/mirror_bot_rviz.rviz)
	
	For the last terminal, go to 
	python3 face_position_bridge_2.py (run in /home/mirrorobot/catkin_ws/src/mirror_bot/src, might need to change Arduino port as needed, ACM0 or ACM1)

	For gpt,
	python3 gpt.py (run in /home/mirrorobot/catkin_ws/src/mirror_bot/src)

##### MirrorBot Play sound #####
To play a sound
	run sound_play.py in vscode (run conda deactivate first)

##### MirrorBot Record Video #####
To record video using Realsense + ROS:
	Make sure realsense node is up (if not: roslaunch realsense2_camera rs_camera.launch align_depth:=true enable_pointcloud:=true)
	In terminal: rosrun mirror_bot realsense_recorder.py
	This will record video to /home folder


##### MirrorBot Integer-to-Action Table
1 - Both mirrors look to the left person
2 - Both mirrors look to the right person
3 - The left mirror will only reflect the left person, the right mirror will connect both people's gazes 
4 - The left mirror will connect both people's gazes, the right mirror will only reflect the right person
5 - The mirror looks down
6 - The mirror will nod up and down (first parameter [bool] - if true, nod the right mirror. second parameter [bool] - if true, nod the left mirror. third parameter [int] - how long the mirror will nod for)
7 - The mirror will pan its head left and right (first parameter [bool] - if true, nod the right mirror. second parameter [bool] - if true, nod the left mirror. third parameter [int] - how long the mirror will nod for)
0 - The mirror will return to default position
