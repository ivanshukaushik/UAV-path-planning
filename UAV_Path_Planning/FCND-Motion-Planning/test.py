import numpy as np
data = np.loadtxt(r'C:\Users\admin\Desktop\UAV_Path_Planning\FCND-Motion-Planning\colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
print(data)
print('##################')
print(data.shape)