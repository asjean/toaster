#!/bin/bash

#maybe call this once so the sh enviroment is setup
gnome-terminal -- sh -c '. ./devel/setup.bash\n; bash'

#roslaunch creates the roscore instance
gnome-terminal -- sh -c 'roslaunch toaster-gazebo gmapping_world.launch; bash'
gnome-terminal -- sh -c 'rosrun teleop_twist_keyboard teleop_twist_keyboard.py; bash'


