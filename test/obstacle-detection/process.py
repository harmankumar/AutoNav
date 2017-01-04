import os
for i in range(1, 1400, 1):
	filename =  str(i).zfill(4) + '.jpg'
        print './obstacle_test floor-video/images/{}'.format(filename)
        os.system(('./obstacle_test floor-video/images/{} floor-video/images-processed/{}'.format(filename, filename) ))
