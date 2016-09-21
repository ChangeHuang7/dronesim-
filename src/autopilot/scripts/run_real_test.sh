

xterm -hold -e "source ~/bebop_ws/devel/setup.sh; roslaunch bebop_driver bebop_node.launch" &
pid1="$!"
xterm -hold -e "source ~/bebop_ws/devel/setup.sh; roslaunch bebop_tools joy_teleop.launch" &
pid2="$!"
xterm -hold -e "source ~/autopilot_ws/devel/setup.sh; roslaunch autopilot test_real_depth.launch sloc:=/experiments/" &
pid3="$!"

read

kill $pid1
kill $pid2
kill -9 $pid3
