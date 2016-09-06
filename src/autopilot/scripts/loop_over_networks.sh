for i in $(seq 40 20 200):
do
  echo "wall_window: $i"
  /home/jay/autopilot_ws/src/autopilot/scripts/loop_over_wall_challenges.sh "wall_10_$(($i-20))" c "winwall_wsize_$i"
done
/home/jay/autopilot_ws/src/autopilot/scripts/loop_over_wall_challenges.sh wall_0010_200 c
