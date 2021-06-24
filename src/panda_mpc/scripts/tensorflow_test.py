#!/home/zheng/venv/bin/python

import os
import sys

import numpy as np
import matplotlib.pyplot as plt


coef = np.array([[10 , 200, -6857.14,  57142.9],
      		[10, -57.1429,  1714.29, -11428.6],
      		[10,  28.5714,        0, -11428.6],
      		[10, -57.1429, -1714.29,  57142.9]])

t = 0.2
l = 3
print(coef[l,0] + coef[l,1]*t + coef[l,2]*pow(t,2) + coef[l,3]*pow(t,3))

interpolation_acc=[]
interpolation_jerk=[]
dt = 0.001
for i in range(200):
	t= i*dt
	if(t<=0.05):
		t = t 
		interpolation_acc.append(coef[0,0] + coef[0,1]*t + coef[0,2]*t*t + coef[0,3]*t*t*t)
		interpolation_jerk.append(coef[0,1] + coef[0,2]*t + coef[0,3]*t*t)
	if (0.05<t<=0.1):	
		t = t - 0.05
		interpolation_acc.append(coef[1,0] + coef[1,1]*t + coef[1,2]*t*t + coef[1,3]*t*t*t)
		interpolation_jerk.append(coef[1,1] + coef[1,2]*t + coef[1,3]*t*t)
	if (0.1<t<=0.15):	
		t = t - 0.1
		interpolation_acc.append(coef[2,0] + coef[2,1]*t + coef[2,2]*t*t + coef[2,3]*t*t*t)
		interpolation_jerk.append(coef[2,1] + coef[2,2]*t + coef[2,3]*t*t)
	if (0.15<t<=0.2):	
		t = t - 0.15
		interpolation_acc.append(coef[3,0] + coef[3,1]*t + coef[3,2]*t*t + coef[3,3]*t*t*t)
		interpolation_jerk.append(coef[3,1] + coef[3,2]*t + coef[3,3]*t*t)

t = np.arange(0.0, 0.2, 0.001)
fig, ax = plt.subplots()
ax.plot(t, interpolation_acc)
#ax.plot(t, interpolation_jerk)
ax.set(xlabel='time (s)', ylabel='voltage (mV)',
       title='About as simple as it gets, folks')
ax.grid()


plt.show()
