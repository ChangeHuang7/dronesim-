# read list of world files in
WORLDDIR="/home/jay/autopilot_ws/src/autopilot/worlds"
WORLDFILES="/home/jay/autopilot_ws/src/autopilot/worlds/oa_challenges_train/*.world"
# whether the trajectory is a success or not is saved log file. 
rm -rf $log
SAVINGDIR="remote_images/depth_estim_expert_poles_2/"
rm -r "/home/jay/data/$SAVINGDIR/*"
logdir="/home/jay/data/$SAVINGDIR/"
log="$logdir/log-continuous-expert"
chmod 775 "/home/jay/data/$SAVINGDIR/*"


xArray=(0.5 2 0 3 -0.5 -1.5 3 4 -2 5)
yArray=(-0.5 -0.5 -2 -2 -4 2 1 -4 -2 -2)

# for file in list launch
# for world in $WORLDFILES
for i in $(seq 0 9)
do
	# FNAME=$(basename ${world}) #get name of the world
	# echo $FNAME
	SLOC="$SAVINGDIR/$i" #cut .world from it
	COMMAND="roslaunch autopilot oa_challenge_depth_estim.launch sloc:=$SLOC current_world:='/oa_world.world' sloc_log:=$log \
	x_0:=${xArray[$i]} y_0:=${yArray[$i]} BA_parameters_path:=/home/jay/autopilot_ws/src/autopilot/parameters/BehaviourArbitration.xml"
	echo $COMMAND

 	START=$(date +%s)                   
	xterm -hold -e $COMMAND &
	pidlaunch=$!
	echo $pidlaunch > "$WORLDDIR/../.pid"
	while kill -0 $pidlaunch; 
	do sleep 0.5
	done

 	END=$(date +%s)
	DIFF=$(( $END - $START ))
	echo "$DIFF" >> "/home/jay/data/$SAVINGDIR/time_info"

	sleep 0.5m
done

# read list of world files in
WORLDDIR="/home/jay/autopilot_ws/src/autopilot/worlds"
WORLDFILES="/home/jay/autopilot_ws/src/autopilot/worlds/oa_challenges_train/*.world"
# whether the trajectory is a success or not is saved log file. 
SAVINGDIR="remote_images/depth_estim_expert_corridor_2/"
rm -r "/home/jay/data/$SAVINGDIR/*"
logdir="/home/jay/data/$SAVINGDIR/"
log="$logdir/log-continuous-expert"
rm -rf $log
chmod 775 "/home/jay/data/$SAVINGDIR/*"

xArray=(4 17 23 30 -8 -1 14 5 -1 30)
yArray=(-19 -14 9 4 -12 -5 -5 1 -1 30)

# for file in list launch
# for world in $WORLDFILES
for i in $(seq 0 9)
do
	# FNAME=$(basename ${world}) #get name of the world
	# echo $FNAME
	SLOC="$SAVINGDIR/$i" #cut .world from it
	COMMAND="roslaunch autopilot oa_challenge_depth_estim.launch sloc:=$SLOC current_world:='/corridor_big.world' sloc_log:=$log \
	x_0:=${xArray[$i]} y_0:=${yArray[$i]}"
	echo $COMMAND

 	START=$(date +%s)                   
	xterm -hold -e $COMMAND &
	pidlaunch=$!
	echo $pidlaunch > "$WORLDDIR/../.pid"
	while kill -0 $pidlaunch; 
	do sleep 0.5
	done

 	END=$(date +%s)
	DIFF=$(( $END - $START ))
	echo "$DIFF" >> "/home/jay/data/$SAVINGDIR/time_info"

	sleep 0.5m
done
