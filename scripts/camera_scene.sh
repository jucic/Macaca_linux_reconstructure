#!/bin/sh

#roslaunch lighthouse eyetracker.launch &

#sleep 2 && roslaunch macaca_tf_setup tf_setup.launch &

sleep 1 && roslaunch usbcam usbcam_scene.launch id0:=2 id1:=1&

sleep 2 && rviz
