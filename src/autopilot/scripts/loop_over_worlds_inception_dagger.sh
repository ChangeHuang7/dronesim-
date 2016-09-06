
# Check saving location
if [ -z "$1" ]
then
	echo "No save location"
	exit
fi
if [ "$1" == "d" ]
then
	echo "No save location"
	exit
fi
# Switch between discrete or continuous supervised control labels...
if [ "$2" == "d" ]
then
    LAUNCHFILE="online_control_dis_dagger.launch"
else

    LAUNCHFILE="online_control_con_dagger.launch"
fi
echo $LAUNCHFILE

#declare -a WORLDFILES=("0005" "0010" "0024" "0026" "0036" "0045" "0055" "0066" "0074" "0083")
#declare -a WORLDFILES=("0005")

WORLDFILES="/home/jay/autopilot_ws/src/autopilot/worlds/oa_challenges_one/*.world"
#WORLDFILES="/home/jay/autopilot_ws/src/autopilot/worlds/oa_challenges_train/*.world"
#WORLDFILES="/home/jay/autopilot_ws/src/autopilot/worlds/oa_challenges_selected/*.world"
#WORLDFILES="/home/jay/autopilot_ws/src/autopilot/worlds/oa_challenges_train_100/*.world"

#8 worlds varying from hard to easy in order to compare quickly the performance of the different control networks
#WORLDFILES="/home/jay/autopilot_ws/src/autopilot/worlds/oa_challenges_test/*.world"

SLOC="/remote_images/set_online"
SLOC_FULL="/home/jay/data$SLOC"

set_name="$1"
mkdir "/home/jay/data/remote_images/$set_name"
chmod 775 "/home/jay/data/remote_images/$set_name"

echo "Removing images from set_online"
rm $SLOC_FULL/* $SLOC_FULL/RGB/* $SLOC_FULL/depth/*

#for WORLD in "${WORLDFILES[@]}"
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
#  	COMMAND="roslaunch autopilot $LAUNCHFILE\
#  	 sloc:=$SLOC current_world:='/oa_challenges_train_100/$WORLD.world'\
#  	  sloc_log:=$log last_control_output:=$lastFile port:=55560"
 	COMMAND="roslaunch autopilot $LAUNCHFILE\
  	 sloc:=$SLOC current_world:='/oa_challenges_train/$WORLD.world'\
	 sloc_log:=$log last_control_output:=$lastFile port:=55560"
 	
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
# Signal pilot_eval to clear inner state
echo "$3" > "/home/jay/data/remote_features/change_network"
echo "send $3"

sleep 30

rm /home/jay/data/remote_features/change_network

#rm -rf /home/jay/data/control_output/* /home/jay/data/remote_features/*