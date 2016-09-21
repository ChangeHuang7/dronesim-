import os
from numpy import mean
import sys
def parseStats(file_path):
	fd = open(file_path, 'r')
	lines = fd.read().split('\n')
	# Parse line 1
	totalAverage = float(lines[0].replace("Total average ", ""))
	# Parse line 2
	# string needs to be cut in 2, at the comma
	commaPos = lines[1].find(',')
	MinRunAv = float(lines[1][0:commaPos].replace('Minimum running average: ', ''))
	MaxRunAv = float(lines[1][commaPos+1:len(lines[1])].replace(' maximum running average: ', ''))

	stats = {"totalAverage": totalAverage, "MinRunAv": MinRunAv, "MaxRunAv": MaxRunAv }

	return stats

def parseSet(world_path):
	os.chdir(world_path)
	# Get all immediate directories
	all_world_dirs = [x for x in os.listdir('.') if os.path.isdir(x)]
	all_stats = []
	# Collect all stats
	for world_dir in all_world_dirs:
		stat_file = world_dir + "/stats"
		stats = parseStats(stat_file)
		all_stats.append(stats)

	# Make lists for each statistic
	MaxRunningAverages = [x['MaxRunAv'] for x in all_stats]
	MinRunningAverages = [x['MinRunAv'] for x in all_stats]
	TotalAverages = [x['totalAverage'] for x in all_stats]

	print "TotalAverages: " + str(mean(TotalAverages))
	print "MinRunningAverages: " + str(mean(MinRunningAverages))
	print "MaxRunningAverages: " + str(mean(MaxRunningAverages))


	stats = {"MaxRunningAverages": MaxRunningAverages, "MinRunningAverages": MinRunningAverages, "TotalAverages": TotalAverages }

dataSet = sys.argv[1] #"var_96_0" #"fc_96"
path = "/home/jay/data/remote_images/" + dataSet

parseSet(path)
