#for i in $(seq 40 20 200):
#do
 # echo "wall_window: $i"
 # /home/jay/autopilot_ws/src/autopilot/scripts/loop_over_wall_challenges.sh "wall_10_$(($i-20))" c "winwall_wsize_$i"
#done
#/home/jay/autopilot_ws/src/autopilot/scripts/loop_over_wall_challenges.sh wall_0010_200 c
# /home/jay/autopilot_ws/src/autopilot/scripts/loop_over_worlds_inception_dagger.sh fc_96 c straight
# /home/jay/autopilot_ws/src/autopilot/scripts/loop_over_worlds_inception_dagger.sh straight_96 c dagger_4G_wsize_300

#/home/jay/autopilot_ws/src/autopilot/scripts/loop_over_worlds_w_depth_estim.sh debug cwall_batchwise

#/home/jay/autopilot_ws/src/autopilot/scripts/loop_over_worlds_inception_dagger.sh lstm_big_96 cwall_batchwise
/home/jay/autopilot_ws/src/autopilot/scripts/loop_over_worlds_ba_test.sh depth_ba_96 
/home/jay/autopilot_ws/src/autopilot/scripts/loop_over_worlds_inception_dagger.sh lstm_big_96 dagger_4G_wsize_300 #


#/home/jay/autopilot_ws/src/autopilot/scripts/loop_over_wall_challenges.sh cwall_train cwall_batchwise 
# /home/jay/autopilot_ws/src/autopilot/scripts/loop_over_wall_challenges.sh cwall_test dagger_4G_wsize_300 
# 
for i in {4..20};
do
	echo "variability test number : $i"
	/home/jay/autopilot_ws/src/autopilot/scripts/loop_over_worlds_inception_dagger.sh var_96_$i dagger_4G_wsize_300
done
echo 'finished'

#remove des folders
#change world source