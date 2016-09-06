#!/usr/bin/python

import xml.etree.ElementTree as ET
import os
from copy import deepcopy

worldDir = "/home/jay/autopilot_ws/src/autopilot/worlds/oa_challenges_train_tmp/"
destDir = "/home/jay/autopilot_ws/src/autopilot/worlds/oa_challenges_train_100/"

worldFiles = os.listdir(worldDir)
for world in worldFiles:
	tree = ET.parse(worldDir + world)
	root = tree.getroot()
	
	lightElements = root[0].findall('light')
	lightEl1 = lightElements[0]
	newLight1 = deepcopy(lightEl1)
	newLight2 = deepcopy(lightEl1)

	model = [x for x in root[0].findall('model') if x.attrib['name'] == 'challenge_w1_0'][0]

	for wall in model.findall('link'):
		wall.find('visual').find('material').find('script').find('name').text = 'Gazebo/PaintedWall'

	wall = [x for x in root[0].findall('model') if x.attrib['name'] == 'wall'][0]
	wall.find('link').find('visual').find('material').find('script').find('name').text = 'Gazebo/PaintedWall'



	newLight1.attrib['name'] = 'startlight1'
	newLight1.attrib['type'] = 'directional'
	newLight1.findall('pose')[0].text = '5 -17 2 0 0 0'
	newLight1.findall('direction')[0].text = '0.1 0 -0.9'
	root[0].append(newLight1)

	newLight2.attrib['name'] = 'startlight2'
	newLight2.attrib['type'] = 'directional'
	newLight2.findall('pose')[0].text = '-5 -17 2 0 0 0'
	newLight2.findall('direction')[0].text = '0.1 0 -0.9'
	root[0].append(newLight2)

	# for light in lightElements:
	# 	light.attrib['type'] = 'directional'
	# 	light.findall('direction')[0].text = '0.1 0 -0.9'

	tree.write((destDir + world))
	print 'Processed ' + world