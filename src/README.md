MirrorBot.py 

Packages required: realsense2_sdk, wg-perception/people

To run code, run each command in a seperate terminal (make sure to run "conda deactivate in each")
roslaunch realsense2_camera rs_camera.launch align_depth:=true enable_pointcloud:=true
roslaunch face_detector face_detector.rs_rgbd.launch
rosrun rviz rviz                                        (optional, but make sure to load /home/yyt/catkin_ws/src/mirror_bot/mirror_bot_rviz.rviz)
python3 face_position_bridge.py                         (run in /home/yyt/catkin_ws/src/mirror_bot/src, might need to change Arduino port as needed, ACM0 or ACM1)

