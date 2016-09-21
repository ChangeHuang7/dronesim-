# Check saving location
if [ -z "$1" ]
then
	echo "No saving location"
	exit
fi

# read list of world files in
WORLDFILES="/home/jay/autopilot_ws/src/autopilot/worlds/sequential_oa/*.world"
#WORLDFILES="/home/jay/autopilot_ws/src/autopilot/worlds/oa_challenges_test/*.world"

SLOC="/remote_images/set_online"
SLOC_FULL="/home/jay/data$SLOC"

set_name="$1"
mkdir "/home/jay/data/remote_images/$set_name"
chmod 775 "/home/jay/data/remote_images/$set_name"

echo "Removing images from set_online"
rm $SLOC_FULL/* $SLOC_FULL/RGB/* $SLOC_FULL/depth/*

# for file in list launch
for WORLD in $WORLDFILES
do

	WORLD=$(basename ${WORLD} | cut -c1-4) #get name of the world
	echo $WORLD
	SLOC_dest="/home/jay/data/remote_images/$set_name/$WORLD"
 	log="/home/jay/data/remote_images/$set_name/log"
 	# Get final entry in control output
 	control_dir="/home/jay/data/control_output/*"
 	#echo $control_dir
 	lastFile=($(ls -rt /home/jay/data/control_output | tail --lines=1))
 	lastFile="${lastFile%.*}"
 	
 	echo $lastFile
	COMMAND="roslaunch autopilot oa_challenge_depth_estim.launch \
 	sloc:=$SLOC current_world:='/oa_challenges_train/$WORLD.world' \
 	sloc_log:=$log last_control_output:=$lastFile port:=55560"
	
 	echo $COMMAND
 	START=$(date +%s)     
  	xterm -hold -e $COMMAND &
  	pidlaunch=$!
  	echo $pidlaunch > "/home/jay/autopilot_ws/src/autopilot/.pid"
  	while kill -0 $pidlaunch; 
  	do sleep 0.5
  	done
	
	# Signal pilot_eval to clear inner state
 	echo "empty file" > "/home/jay/data/remote_features/clear_memory"
 	
 	# When killed, copy everything to other folder
	mkdir -p $SLOC_dest
  	chmod 775 $SLOC_dest
  	echo "Copying to dagger folder $set_name"
  	cp -r $SLOC_FULL/* $SLOC_dest
  	echo "Removing images from set_online"
 	# Remove everything in the folder
 	rm $SLOC_FULL/* $SLOC_FULL/RGB/* $SLOC_FULL/depth/*

 	END=$(date +%s)
	DIFF=$(( $END - $START ))
	echo "$DIFF" >> "/home/jay/data/remote_images/$set_name/time_info"
 	
 	sleep 30
 	
 	#make file free again
 	rm /home/jay/data/remote_features/clear_memory
done
if 

if [ -z "$2" ]
then
exit
fi

# Signal pilot_eval to clear inner state
echo "$2" > "/home/jay/data/remote_features/change_network"
echo "send $2"

sleep 30

rm /home/jay/data/remote_features/change_network
