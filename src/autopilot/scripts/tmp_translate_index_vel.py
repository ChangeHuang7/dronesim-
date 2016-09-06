
# This expects an input "pos" that is between 0 and disc_factor - 1. It corresponds to the index of the bit that is activated.
def translate_to_vel(pos, disc_factor):
	# The size of the step
	b = 2.0/(disc_factor-1);
	# If test is required because the discretized 0 is a seperate state
	if pos < (disc_factor-1)/2:
		# Negative value
		return (-1 + b*pos)
	else:
		return (-1 + b*(pos+1))

def translate_index(index, disc_factor):
	if index == 0:
		# only go forward
		z_linear = 0
		z_angular = 0
	elif index < disc_factor:
		# Go up or down
		# Compensate for index 0 being go forward
		position = index - 1
		z_linear = translate_to_vel(position, disc_factor)
		z_angular = 0
	else:
		position = index - disc_factor
		z_angular = translate_to_vel(position, disc_factor)
		z_linear = 0

	print "Index: " + str(index)
	print "Linear vel: " + str(z_linear)
	print "Angular vel: " + str(z_angular)
	
# How many steps the signal is discretized
disc_factor = 21;

# index is the Index of the bit that is active.
# index = 0 --> go straight
# 1 --> 20 --> go up or down
# 21 -->  41 --> go left or right (if 21 discretized values were used)

for index in range(disc_factor*2-1):
	translate_index(index, disc_factor)