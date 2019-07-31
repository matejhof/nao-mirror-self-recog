import numpy as np
import sys
import matplotlib.pyplot as plt


skin_map = {}
map_fh = open('/home/vojta/code-nao-simulation/gazebo9/catkin_ws/src/my_executables/scripts/coordinate-maps/highres-head.txt', 'r')
for i in range(240):
    k = str(i) + '_' + str(i + 2)
    skin_map[k] = [float(x) for x in map_fh.readline().strip().split(" ")]
map_fh.close()


x_values =[]
y_values = []
for i in skin_map:
	x_values.append(skin_map[i][0])
	y_values.append(skin_map[i][1])

plt.plot(x_values,y_values,'ro',color = 'k')
plt.show()



