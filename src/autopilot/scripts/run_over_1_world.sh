
WORLD="0055"
SLOC="/remote_images/dagger1/$WORLD" #cut .world from it
log="$SLOC/log"
COMMAND="roslaunch autopilot online_control_con_dagger.launch sloc:=$SLOC current_world:='/oa_challenges_train/$WORLD.world' sloc_log:=$log"
echo $COMMAND
            
xterm -hold -e $COMMAND &
pidlaunch=$!
echo $pidlaunch > "/home/jay/autopilot_ws/src/autopilot/.pid"
while kill -0 $pidlaunch; 
do sleep 0.5
done
