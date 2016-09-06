# read list of world files in
WORLDDIR="/home/jay/autopilot_ws/src/autopilot/worlds"
WORLDFILES="/home/jay/autopilot_ws/src/autopilot/worlds/oa_challenges_train_100_tmp/*.world"
# whether the trajectory is a success or not is saved log file. 
logdir='/home/jay/data/remote_images/depth_estimation_expert_2_sept_part_2/'
log="$logdir/log-continuous-expert"
# rm -rf $log
# SAVINGDIR="remote_images/debug_2"
SAVINGDIR="remote_images/depth_estimation_expert_2_sept_part_2"


mkdir "/home/jay/data/$SAVINGDIR/"
rm -r "/home/jay/data/$SAVINGDIR/*"
chmod 775 "/home/jay/data/$SAVINGDIR/*"

# for file in list launch
for world in $WORLDFILES
# for ((x=14; x<96; x++));
do
	# echo "${WORLDFILES[x]}"
	FNAME=$(basename ${world}) #get name of the world
	echo $FNAME
	SLOC="$SAVINGDIR/$(basename ${world} | cut -c1-4)" #cut .world from it
	COMMAND="roslaunch autopilot oa_challenge_depth_estim.launch sloc:=$SLOC current_world:='/oa_challenges_train_100_tmp/$FNAME' sloc_log:=$log port:=55560"
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
	echo "$DIFF" >> "$logdir/time_info"
	sleep 0.5m
done
