#for i in $(seq 40 20 200):
#do
 # echo "wall_window: $i"
 # /home/jay/autopilot_ws/src/autopilot/scripts/loop_over_wall_challenges.sh "wall_10_$(($i-20))" c "winwall_wsize_$i"
#done
#/home/jay/autopilot_ws/src/autopilot/scripts/loop_over_wall_challenges.sh wall_0010_200 c
/home/jay/autopilot_ws/src/autopilot/scripts/loop_over_worlds_inception_dagger.sh fc_96 c straight
/home/jay/autopilot_ws/src/autopilot/scripts/loop_over_worlds_inception_dagger.sh straight_96 c dagger_4G_wsize_300
for i in {0..1};
do
	echo "variability test number : $i"
	/home/jay/autopilot_ws/src/autopilot/scripts/loop_over_worlds_inception_dagger.sh var_96_$i c dagger_4G_wsize_300
done
echo 'finished'
