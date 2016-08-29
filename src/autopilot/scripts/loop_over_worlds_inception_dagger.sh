
# declare -a WORLDFILES=("0005" "0010" "0024" "0026" "0036" "0045" "0055" "0066" "0074" "0083")
declare -a WORLDFILES=("0005")
SLOC="/remote_images/set_online"
SLOC_FULL="/home/jay/data$SLOC"

set_name="$1"
mkdir "/home/jay/data/remote_images/$set_name"
chmod 775 "/home/jay/data/remote_images/$set_name"

echo "Removing images from set_online"
rm $SLOC_FULL/* $SLOC_FULL/RGB/* $SLOC_FULL/depth/*

for WORLD in "${WORLDFILES[@]}"
do
	echo $WORLD
	SLOC_dest="/home/jay/data/remote_images/dagger3/$WORLD" #cut .world from it
	log="$SLOC/log"
	# Get final entry in control output
	control_dir="/home/jay/data/control_output/*"
	echo $control_dir
	lastFile=($(ls -rt /home/jay/data/control_output | tail --lines=1))
	lastFile="${lastFile%.*}"
	echo $lastFile
	COMMAND="roslaunch autopilot online_control_con_dagger.launch\
	 sloc:=$SLOC current_world:='/oa_challenges_train/$WORLD.world'\
	  sloc_log:=$log last_control_output:=$lastFile"
	echo $COMMAND
	            
	xterm -hold -e $COMMAND &
	pidlaunch=$!
	echo $pidlaunch > "/home/jay/autopilot_ws/src/autopilot/.pid"
	while kill -0 $pidlaunch; 
	do sleep 0.5
	done

	# When killed, copy everything to other folder
	mkdir -p $SLOC_dest
	chmod 775 $SLOC_dest
	echo "Copying to dagger folder"
	cp -r $SLOC_FULL/* $SLOC_dest
	echo "Removing images from set_online"
	# Remove everything in the folder
	rm $SLOC_FULL/* $SLOC_FULL/RGB/* $SLOC_FULL/depth/*
done