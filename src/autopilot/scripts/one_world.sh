# read list of world files in
WORLDDIR="/home/jay/autopilot_ws/src/autopilot/worlds"
WORLDFILES="/home/jay/autopilot_ws/src/autopilot/worlds/oa_challenges_train/*.world"
# whether the trajectory is a success or not is saved log file. 
logdir='/home/jay/autopilot_ws/src/autopilot'
log="$logdir/log-continuous-expert"
rm -rf $log	

world="0000.world"

	FNAME=$(basename ${world}) #get name of the world
	echo $FNAME
	#SLOC=$(basename ${world} | cut -c1-4) #cut .world from it
	SLOC="test"
	COMMAND="roslaunch autopilot oa_challenge_test.launch current_world:='/oa_challenges_train/$FNAME' sloc:=$SLOC"
	xterm -hold -e $COMMAND &
	pidlaunch=$!
	echo $pidlaunch > "$WORLDDIR/../.pid"
	

