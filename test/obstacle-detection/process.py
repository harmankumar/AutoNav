import os
for i in range(1, 73, 1):
	filename = 'out' + str(i).zfill(4) + '.jpg'
	os.system(('./obstacle_test floor_video/obstacle_avoidance_2/{} floor_video/obstacle_avoidance_2_processed/{}'.format(filename, filename) ))
