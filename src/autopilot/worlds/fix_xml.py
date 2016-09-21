#!/usr/bin/python

import xml.etree.ElementTree as ET
import os

worldDir = "/home/jay/autopilot_ws/src/autopilot/worlds/oa_challenges_train_100plus/"
destDir = "/home/jay/autopilot_ws/src/autopilot/worlds/oa_challenges_train_tmp/"
worldFiles = os.listdir(worldDir)
for world in worldFiles:
	f = open(worldDir + world, 'r')
	lineList = []
	for line in f:
		lineList.append(line)
	f.close()
	lineList[0] = lineList[0].replace('<?xml version=1?>', '<?xml version="1"?>')
	fullString = '\n'.join(lineList)

	fw = open(destDir + world, 'w')
	fw.write(fullString)
	fw.close()