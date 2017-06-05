import matplotlib.pyplot as plt
import numpy as np
import math


def generate_gaussian(mu, sigma, N):
    return np.random.normal(mu, sigma, N)

def gaussian_noice_added(x, sigma):
    r = []
    for i in x:
        r.extend(generate_gaussian(i, sigma, 1))
    return r

def relative_noice(relative, x):
    r = []
    cumulated = 0
    for i,j in zip(relative,x):
        cumulated += i
        r.append(j+cumulated)
    return r

def mixed(relative, absolute, sigma_relative, sigma_absolute, c=None):
    phi = sigma_absolute/sigma_relative
    if c is None:
        c = phi*phi/(phi*phi+1)
        c = 0.73
    r = [absolute[0]]
    for i in range(1,len(relative)):
        r.append((1-c)*absolute[i]+c*relative[i]+c*r[i-1])
    return r

N = 500
t = np.arange(0, N, 1)
x_base = [float(i)/5.0 for i in range(N//2)]
x_base.extend([float(N//2-1-i)/5.0 for i in range(N//2)])

odometry_error = gaussian_noice_added([0 for i in range(N)], 0.6)
odometry = relative_noice(odometry_error, x_base)
beacon = gaussian_noice_added(x_base, 3.0)
corrected = mixed(odometry_error, beacon, 0.3, 3.0)

plt.figure(1)
plt.plot(t, beacon, 'k', color='cyan')
plt.plot(t, odometry, 'k', color='red')
plt.plot(t, x_base, 'k', color='black')
plt.plot(t, corrected, 'k', color='green')
plt.show()

print("Mean error and standard deviation beacon : "+str(abs((np.array(x_base)-np.array(beacon)).mean()))+" "+str((np.array(x_base)-np.array(beacon)).std()))
print("Mean error and standard deviation odometry : "+str(abs((np.array(x_base)-np.array(odometry)).mean()))+" "+str((np.array(x_base)-np.array(odometry)).std())+" and just the relative error : "+str(abs(np.array(odometry_error).mean()))+" "+str(np.array(odometry_error).std()))
print("Mean error and standard deviation corrected : "+str(abs((np.array(x_base)-np.array(corrected)).mean()))+" "+str((np.array(x_base)-np.array(corrected)).std()))

var = []
mean = []
index = 0
min = -1
for i in np.arange(0.1, 0.9, 0.01):
    corrected = mixed(odometry_error, beacon, 0.3, 30, i)
    v = (np.array(x_base)-np.array(corrected)).std()
    m = abs((np.array(x_base)-np.array(corrected)).mean())
    var.append(v)
    mean.append(m)

    if v*v+m*m<min or min<0:
        min = v*v+m*m
        index = i

print("Experimental min found is "+str(min)+" at "+str(index))

t = np.arange(0.1, 0.9, 0.01)
plt.plot(t, var, 'k', color='red')
plt.plot(t, mean, 'k', color='green')
plt.plot([index, index], [0, math.sqrt(min)])
plt.show()
