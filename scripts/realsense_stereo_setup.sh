#!/bin/bash

# Taken from
# https://answers.ros.org/question/51474/can-i-run-a-bash-script-using-roslaunch/?answer=157051?answer=157051#post-id-157051

# This works for the D415 camera
# rosrun dynamic_reconfigure dynparam set /camera/stereo_module \
# "{'exposure': 5000,
# 'gain' : 100,
# 'visual_preset' : 0,
# 'frames_queue_size' : 8,
# 'laser_power' : 0.0,
# 'inter_cam_sync_mode' : 0.0,
# 'enable_auto_exposure' : 0,
# 'enable_auto_white_balance' : 1, #
# 'error_polling_enabled' : 1,
# 'output_trigger_enabled' : 0,
# 'global_time_enabled' : 1
# }"

#D435
rosrun dynamic_reconfigure dynparam set /camera/stereo_module \
"{'exposure': 500,
'gain' : 100,
'visual_preset' : 0,
'frames_queue_size' : 8,
'laser_power' : 0.0,
'inter_cam_sync_mode' : 0.0,
'enable_auto_exposure' : 0,
'error_polling_enabled' : 1,
'output_trigger_enabled' : 0,
'global_time_enabled' : 1
}"



