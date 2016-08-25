# read list of world files in
WORLDDIR="/home/jay/autopilot_ws/src/autopilot/worlds"
WORLDFILES="/home/jay/autopilot_ws/src/autopilot/worlds/oa_challenges_train/*.world"
# whether the trajectory is a success or not is saved log file. 
logdir='/home/jay/autopilot_ws/src/autopilot'
log="$logdir/log-continuous-expert"
rm -rf $log
SAVINGDIR="remote_images/debug_2"
rm -r "/home/jay/data/$SAVINGDIR/*"
chmod 775 "/home/jay/data/$SAVINGDIR/*"

# for file in list launch
for world in $WORLDFILES
do
	FNAME=$(basename ${world}) #get name of the world
	echo $FNAME
	SLOC="$SAVINGDIR/$(basename ${world} | cut -c1-4)" #cut .world from it
	COMMAND="roslaunch autopilot oa_challenge.launch sloc:=$SLOC current_world:='/oa_challenges_train/$FNAME' sloc_log:=$log"
	echo $COMMAND
                
	xterm -hold -e $COMMAND &
	pidlaunch=$!
	echo $pidlaunch > "$WORLDDIR/../.pid"
	while kill -0 $pidlaunch; 
	do sleep 0.5
	done
	sleep 0.5m
done
