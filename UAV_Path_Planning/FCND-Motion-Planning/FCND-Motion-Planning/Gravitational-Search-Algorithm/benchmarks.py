# -*- coding: utf-8 -*-
"""
Python code of Gravitational Search Algorithm (GSA)
Reference: Rashedi, Esmat, Hossein Nezamabadi-Pour, and Saeid Saryazdi. "GSA: a gravitational search algorithm." 
           Information sciences 179.13 (2009): 2232-2248.	

Coded by: Mukesh Saraswat (saraswatmukesh@gmail.com), Himanshu Mittal (emailid: himanshu.mittal224@gmail.com) and Raju Pal (emailid: raju3131.pal@gmail.com)
The code template used is similar given at link: https://github.com/7ossam81/EvoloPy and matlab version of GSA at mathworks.

 -- Purpose: Defining the benchmark function code 
              and its parameters: function Name, lowerbound, upperbound, dimensions

Code compatible:
 -- Python: 2.* or 3.*
"""

import numpy
import math


#determinant of matrix a
def det(a):
    return a[0][0]*a[1][1]*a[2][2] + a[0][1]*a[1][2]*a[2][0] + a[0][2]*a[1][0]*a[2][1] - a[0][2]*a[1][1]*a[2][0] - a[0][1]*a[1][0]*a[2][2] - a[0][0]*a[1][2]*a[2][1]

#unit normal vector of plane defined by points a, b, and c
def unit_normal(a, b, c):
    x = det([[1,a[1],a[2]],
             [1,b[1],b[2]],
             [1,c[1],c[2]]])
    y = det([[a[0],1,a[2]],
             [b[0],1,b[2]],
             [c[0],1,c[2]]])
    z = det([[a[0],a[1],1],
             [b[0],b[1],1],
             [c[0],c[1],1]])
    magnitude = (x**2 + y**2 + z**2)**.5
    return (x/magnitude, y/magnitude, z/magnitude)


#dot product of vectors a and b
def dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


#cross product of vectors a and b
def cross(a, b):
    x = a[1] * b[2] - a[2] * b[1]
    y = a[2] * b[0] - a[0] * b[2]
    z = a[0] * b[1] - a[1] * b[0]
    return (x, y, z)


#area of polygon poly
def area(poly):
    if len(poly) < 3: # not a plane - no area
        return 0

    total = [0, 0, 0]
    for i in range(len(poly)):
        vi1 = poly[i]
        if i is len(poly)-1:
            vi2 = poly[0]
        else:
            vi2 = poly[i+1]
        prod = cross(vi1, vi2)
        total[0] += prod[0]
        total[1] += prod[1]
        total[2] += prod[2]
    result = dot(total, unit_normal(poly[0], poly[1], poly[2]))
    return abs(result/2)

def grid():
  data = numpy.loadtxt(r'C:\Users\admin\Desktop\UAV_Path_Planning\FCND-Motion-Planning\colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

  TARGET_ALTITUDE = 20
  SAFETY_DISTANCE = 5  
  #Define a grid for a particular altitude and safety margin around obstacles
  grid, north_offset, east_offset, alt_offset= planning_utils.create_3d_grid(data, SAFETY_DISTANCE)
  return grid

    
def F1(x):
  """ Sphere Function """
  xi = [0, 0, 0]
  xf = [100, 100, 100]
  print(x, 'fitnessfunction')
  distances = []
  num_way = 6
  
  
  s1 = (x[0] - xi[0]) ** 2 + (x[1] - xi[1]) ** 2 + (x[2] - xi[2]) ** 2
  s2 = (x[3] - x[0]) ** 2 + (x[4] - x[1]) ** 2 + (x[5] - x[2]) ** 2
  s3 = (x[6] - x[3]) ** 2 + (x[7] - x[4]) ** 2 + (x[8] - x[5]) ** 2
  s4 = (x[9] - x[6]) ** 2 + (x[10] - x[7]) ** 2 + (x[11] - x[8]) ** 2
  s5 = (x[12] - x[9]) ** 2 + (x[13] - x[10]) ** 2 + (x[14] - x[11]) ** 2
  s6 = (x[15] - x[12]) ** 2 + (x[16] - x[13]) ** 2 + (x[17] - x[14]) ** 2
  s7 = (xf[0] - x[15]) ** 2 + (xf[1] - x[16]) ** 2 + (xf[2] - x[17]) ** 2

  z_mean = (x[2] + x[5] + x[8] + x[11] + x[14] + x[17]) / num_way
  z = (x[2] - z_mean) ** 2 + (x[5] - z_mean) ** 2 + (x[8] - z_mean) ** 2 + (x[11] - z_mean) ** 2 + (x[14] - z_mean) ** 2 + (x[17] - z_mean) ** 2
  z = numpy.sqrt(z/num_way)
  s = s1 + s2 + s3 + s4 + s5 + s6 + s7 + z

  


  #s=numpy.sum(x**2)
  

  return s




def getFunctionDetails(a):
  # [name, lb, ub, dim]
  num_Way = 6
  param = {  0: ["F1",-100,100, num_Way * 3],
            1: ["F2", 0, 100, 15]}
  return param.get(a, "nothing")



