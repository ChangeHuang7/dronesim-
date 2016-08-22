# read list of world files in
WORLDDIR="/home/jay/autopilot_ws/src/autopilot/worlds"
WORLDFILES="/home/jay/autopilot_ws/src/autopilot/worlds/auto_generated/*.world"
# whether the trajectory is a success or not is saved log file. 
rm -rf /home/jay/autopilot_ws/src/autopilot/log.txt

# for file in list launch
for world in $WORLDFILES
do
	FNAME=$(basename ${world}) #get name of the world
	echo $FNAME
	SLOC=$(basename ${world} | cut -c1-4) #cut .world from it
	COMMAND="roslaunch autopilot oa_challenge.launch sloc:='$SLOC' current_world:='/auto_generated/$FNAME' height_behavior:='high'"
	xterm -hold -e $COMMAND &
	pidlaunch=$!
	echo $pidlaunch > "$WORLDDIR/../.pid"
	while kill -0 $pidlaunch; 
	do sleep 0.5
	done
	sleep 1m
done
for world in $WORLDFILES
do
	FNAME=$(basename ${world}) #get name of the world
	echo $FNAME
	SLOC=$(basename ${world} | cut -c1-4) #cut .world from it
	COMMAND="roslaunch autopilot oa_challenge.launch sloc:='1$SLOC' current_world:='/auto_generated/$FNAME' height_behavior:='low'"
	xterm -hold -e $COMMAND &
	pidlaunch=$!
	echo $pidlaunch > "$WORLDDIR/../.pid"
	while kill -0 $pidlaunch; 
	do sleep 0.5
	done
	sleep 1m	
done
