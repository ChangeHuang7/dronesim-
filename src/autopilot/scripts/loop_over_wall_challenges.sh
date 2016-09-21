# Check saving location
if [ -z "$1" ]
then
	echo "No save location"
	exit
fi

if [ "$1" == "cwall_train" ]
then 
  WORLDFILES="/home/jay/autopilot_ws/src/autopilot/worlds/wall_challenges_train/*.world"
elif [ "$1" == "cwall_test" ]
then 
  WORLDFILES="/home/jay/autopilot_ws/src/autopilot/worlds/wall_challenges_test/*.world"
fi
echo $WORLDFILES
# read list of world files in
#WORLDDIR="/home/jay/autopilot_ws/src/autopilot/worlds"
#WORLDFILES="/home/jay/autopilot_ws/src/autopilot/worlds/wall_challenges_train/*.world"

#WORLDFILES="/home/jay/autopilot_ws/src/autopilot/worlds/wall_challenges_one/*.world"

# whether the trajectory is a success or not is saved log file. 
#logdir='/home/jay/autopilot_ws/src/autopilot'
SLOC="/remote_images/set_online"
SLOC_FULL="/home/jay/data$SLOC"

set_name="$1"
mkdir "/home/jay/data/remote_images/$set_name"
chmod 775 "/home/jay/data/remote_images/$set_name"

echo "Removing images from set_online"
rm $SLOC_FULL/* $SLOC_FULL/RGB/* $SLOC_FULL/depth/*

declare -a turning_array=('0.1' '-0.1' '3' '3.2')
# spawn dir

declare -a flying_array=('3.14' '0') 
# direction ==> prefered direction && evaluation mode && spawn_dir
i=0
for turning_direction in "${turning_array[@]}";
do
    log="/home/jay/data/remote_images/$set_name/log"
    ((i++))
    for WORLD in $WORLDFILES
    do
        WORLD=$(basename ${WORLD} | cut -c1-4) #get name of the world
        echo $WORLD
        
        SLOC_dest="/home/jay/data/remote_images/$set_name/${WORLD}_$i"
 	
 	# Get final entry in control output
 	control_dir="/home/jay/data/control_output/*"
 	lastFile=($(ls -rt /home/jay/data/control_output | tail --lines=1))
 	lastFile="${lastFile%.*}"
 	echo $lastFile
 	
        # Select the correct direction in which to fly
        if [ $turning_direction == '0.1' ] || [ $turning_direction == '-0.1' ]; then
            flying_direction_current=${flying_array[0]}
            MODE=2
        else
            flying_direction_current=${flying_array[1]}
            MODE=1
        fi
        echo "Flying direction: $flying_direction_current"
	echo "Turning direction: $MODE"

        COMMAND="roslaunch autopilot wall_challenge.launch sloc:=${SLOC}\
        current_world:='/wall_challenges_train/$WORLD.world'\
        steering_direction:=$flying_direction_current\
        sloc_log:=$log last_control_output:=$lastFile\
        spawn_dir:=$turning_direction MODE:=$MODE"
        
        echo $COMMAND
        
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
 	echo "Copying to dagger folder"
 	cp -r $SLOC_FULL/* $SLOC_dest
 	echo "Removing images from set_online"
 	
 	# Remove everything in the folder
 	rm $SLOC_FULL/* $SLOC_FULL/RGB/* $SLOC_FULL/depth/*
 	
        sleep 30
        
 	#make file free again
 	rm /home/jay/data/remote_features/clear_memory
    done
done

# Signal pilot_eval to clear inner state
echo "$3" > "/home/jay/data/remote_features/change_network"
echo "send $3"

sleep 30

rm /home/jay/data/remote_features/change_network

#rm -rf /home/jay/data/control_output/* /home/jay/data/remote_features/*
#echo 'finished'