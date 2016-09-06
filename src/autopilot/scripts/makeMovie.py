#!/usr/bin/python

import os

path = "/home/jay/data/remote_images/dagger_lstm_depth/0012/"

list_depth = os.listdir(path + "depth")
list_RGB = os.listdir(path + "RGB")

print len(list_RGB)
print len(list_depth)

ratio = len(list_depth) / float(len(list_RGB))

print ratio

os.chdir(path)
os.system("ffmpeg \
	-framerate 25 -pattern_type glob -i 'RGB/*.jpg' -c:v libx264 -r 30 rgb.mp4")
os.system("ffmpeg \
	-framerate " + str(25*ratio) + " -pattern_type glob -i 'depth/*.jpg' -c:v libx264 -r 30 depth.mp4")

command = "ffmpeg -i rgb.mp4 -i depth.mp4 -filter_complex \"nullsrc=size=1280x480 [base]; [0:v] setpts=PTS-STARTPTS, scale=640x480 [upper]; [1:v] setpts=PTS-STARTPTS, scale=640x480 [lower]; [base][upper] overlay=shortest=1 [tmp1]; [tmp1][lower] overlay=shortest=1:x=640\" -c:v libx264 output.mkv"
print command
os.system(command)
