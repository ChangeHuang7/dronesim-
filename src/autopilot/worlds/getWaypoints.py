#!/usr/bin/python

import xml.etree.ElementTree as ET
import sys

if len(sys.argv) != 4:
	print("Usage: getWaypoints.py [world_with_waypoints] [save_destination_world] [save_destination_waypoint_file]")
	exit()

world = sys.argv[1]
dest = sys.argv[2]
waypointFile = sys.argv[3]

# world = "/home/tom/Desktop/test_esat_2.world"
# dest = "/home/tom/Desktop/test_esat_3.world"

# waypointFile = "/home/tom/Desktop/waypoints"

tree = ET.parse(world)
root = tree.getroot()
models = root[0].findall("model")
waypointString = []
for waypoint in (x for x in models if "unit_cylinder" in x.attrib["name"]):
# for models in:
		wayID = int(waypoint.attrib["name"].replace("unit_cylinder_", ""))
		pose = waypoint.find("pose").text
		poseVec = pose.split(" ")
		poseVec
		waypointString.append(str(wayID) + " " + " ".join(poseVec[0:3]))
		root[0].remove(waypoint)

tree.write(dest)
print "\n".join(waypointString)

fd = open(waypointFile, 'w')
fd.write("\n".join(waypointString))
fd.close()
