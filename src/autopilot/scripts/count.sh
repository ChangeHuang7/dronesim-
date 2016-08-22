#run twice once for 0 ~ flying high and once 1 ~ for flying low
for i in $(seq 0 1 $END);
do
echo $i
folders="/home/jay/data/remote_images/$i*"
failure_destination="/home/jay/data/remote_images/failures"
for folder in $folders
do
  count=$(ls -l "$folder/RGB/" | wc -l)
  if [ "$count" -lt "700" ]
  then
	echo "$(basename $folder) $count bump" 
	bumps=$((bumps + 1 ))
	mv $folder "$failure_destination/$(basename $folder)"
	#echo "mv $folder $failure_destination/$(basename $folder)"	
  fi
  if [[ "$count" -gt "700" && "$count" -lt "3000" ]]
  then
	echo "$(basename $folder) $count succes"
	successes=$((successes+1))
  fi
  if [ "$count" -gt "3000" ]
  then
	echo "$(basename $folder) $count stuck"
	stucks=$((stucks + 1))
	mv $folder "$failure_destination/$(basename $folder)"
  fi
done
done
total=$((bumps+stucks+successes))
echo "$bumps / $total bumps; $successes / $total success; $stucks / $total stuck;"

