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

SLOC="/remote_images/set_online"
SLOC_FULL="/home/jay/data$SLOC"

set_name="$1"
mkdir "/home/jay/data/remote_images/dagger_esat/$set_name"
chmod 775 "/home/jay/data/remote_images/dagger_esat/$set_name"

echo "Removing images from set_online"
rm $SLOC_FULL/* $SLOC_FULL/RGB/* $SLOC_FULL/depth/*

SLOC_dest="/home/jay/data/remote_images/dagger_esat/$set_name"
log="/home/jay/data/remote_images/dagger_esat/$set_name/log"

control_dir="/home/jay/data/control_output/*"

lastFile=($(ls -rt /home/jay/data/control_output | tail --lines=1))
lastFile="${lastFile%.*}"
echo $lastFile
  	
COMMAND="roslaunch autopilot esat_corridor_test.launch \
  sloc:=$SLOC current_world:='/esat_corridor_waypoints/corridor_esat_challenge.world' \
  sloc_log:=$log last_control_output:=$lastFile port:=55560"

echo $COMMAND
 	            
xterm -hold -e $COMMAND &
pidlaunch=$!
echo $pidlaunch > "/home/jay/autopilot_ws/src/autopilot/.pid"
while kill -0 $pidlaunch; 
do sleep 0.5
done

# Signal pilot_eval to clear inner state
#echo "empty file" > "/home/jay/data/remote_features/clear_memory"

# When killed, copy everything to other folder
mkdir -p $SLOC_dest
chmod 775 $SLOC_dest
echo "Copying to dagger folder: $set_name"
cp -r $SLOC_FULL/* $SLOC_dest
echo "Removing images from set_online"
# Remove everything in the folder
rm $SLOC_FULL/* $SLOC_FULL/RGB/* $SLOC_FULL/depth/*

#sleep 2m

#make file free again
#rm /home/jay/data/remote_features/clear_memory