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
        c = phi/(phi+1)
        c = 0.69
        #c = sigma_absolute/(sigma_absolute+sigma_relative)
    r = [absolute[0]]
    for i in range(1,len(relative)):
        r.append((1-c)*absolute[i]+c*relative[i]+c*r[i-1])
    return r

def simu_get_position_codeuse_et_triangulation(relative, absolute, sigma_relative, sigma_absolute):
    simu = absolute[:]

    for i in range(1, len(absolute)):
        simu[i] = (sigma_relative * absolute[i] + sigma_absolute * (relative[i] + simu[i-1])) / (sigma_absolute + sigma_relative)

    return simu

N = 500
t = np.arange(0, N, 1)
x_base = [float(i)/2.0 for i in range(N//2)]
x_base.extend([float(N//2-1-i)/2.0 for i in range(N//2)])

sigma_relative = 1.4
sigma_absolute = 5.0

mean_ju = []
mean_nico = []
std_ju = []
std_nico = []
for i in range(1000):
    beacon = gaussian_noice_added(x_base, sigma_absolute)
    odometry_error = generate_gaussian(0, sigma_relative, N)
    odometry_measured = np.concatenate((np.array([0.0]), np.array(x_base[1:])-np.array(x_base[:-1])))+np.array(odometry_error)
    odometry = relative_noice(odometry_error, x_base)

    corrected_ju = mixed(odometry_measured, beacon, sigma_relative, sigma_absolute)
    corrected_nico = simu_get_position_codeuse_et_triangulation(odometry_measured, beacon, sigma_relative, sigma_absolute)

    mean_ju.append(abs((np.array(x_base)-np.array(corrected_ju)).mean()))
    mean_nico.append(abs((np.array(x_base)-np.array(corrected_nico)).mean()))
    std_ju.append((np.array(x_base)-np.array(corrected_ju)).std())
    std_nico.append((np.array(x_base)-np.array(corrected_nico)).std())

print("Mean of mean error and deviation (julien) : "+str(np.array(mean_ju).mean())+" "+str(np.array(std_ju).mean()))
print("Mean of mean error and deviation (nico) : "+str(np.array(mean_nico).mean())+" "+str(np.array(std_nico).mean()))


plt.figure(1)
plt.plot(t, beacon, 'k', color='cyan')
plt.plot(t, odometry, 'k', color='red')
plt.plot(t, odometry_measured, 'k', color='grey')
plt.plot(t, x_base, 'k', color='black')
plt.plot(t, corrected_ju, 'k', color='green')
plt.plot(t, corrected_nico, 'k', color='orange')
plt.show()

print("")
print("And result on last stage :")
print("Mean error and standard deviation beacon : "+str(abs((np.array(x_base)-np.array(beacon)).mean()))+" "+str((np.array(x_base)-np.array(beacon)).std()))
print("Mean error and standard deviation odometry : "+str(abs((np.array(x_base)-np.array(odometry)).mean()))+" "+str((np.array(x_base)-np.array(odometry)).std())+" and just the relative error : "+str(abs(np.array(odometry_error).mean()))+" "+str(np.array(odometry_error).std()))
print("Mean error and standard deviation corrected (julien) : "+str(abs((np.array(x_base)-np.array(corrected_ju)).mean()))+" "+str((np.array(x_base)-np.array(corrected_ju)).std()))
print("Mean error and standard deviation corrected (nico) : "+str(abs((np.array(x_base)-np.array(corrected_nico)).mean()))+" "+str((np.array(x_base)-np.array(corrected_nico)).std()))

var = []
mean = []
index = 0
min = -1
for i in np.arange(0.1, 0.9, 0.01):
    corrected = mixed(odometry_error, beacon, sigma_relative, sigma_absolute, i)
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
