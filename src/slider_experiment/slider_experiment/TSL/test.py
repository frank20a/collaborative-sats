import matplotlib.pyplot as plt
import numpy as np
import sys, os
from numpy import pi, sin, cos
from scipy.signal import sawtooth
from ament_index_python import get_package_share_directory
from parameters import *

sys.path.insert(1, os.path.join(get_package_share_directory('slider_experiment'), 'python_build/tsl_optimizer'))
import tsl_optimizer as optimizer
solver = optimizer.solver()

# create a time array from 0 to 10 with ts = 0.1
ts = np.arange(0, 4, 0.01)

r = 0.7
th = 2*pi/4 * ts

Fx = r*cos(3*th)/0.9
Fy = r*sin(2*th)/1.1
tau = 0.2*sawtooth(th*4, 0.3)

# Create a 3x1 plot with Fx, Fy and tau
plt.figure(1)
plt.suptitle("Disired Force/Torque")
plt.subplot(311)
plt.plot(ts, Fx)
plt.ylabel('Fx')
plt.xlabel('t')
plt.subplot(312)
plt.plot(ts, Fy)
plt.ylabel('Fy')
plt.xlabel('t')
plt.subplot(313)
plt.plot(ts, tau)
plt.ylabel('tau')
plt.xlabel('t')

out = []
for x, y, t in zip(Fx, Fy, tau):
    tmp = solver.run(p = [x, y, t]).solution
    out.append(tmp)
out = np.array(out)

# Create a 8 x 1 plot of out
plt.figure(2)
plt.suptitle("Thruster force")
plt.subplot(811)
plt.plot(ts, out[:, 0])
plt.ylabel('T11')
plt.xlabel('t')
plt.subplot(812)
plt.plot(ts, out[:, 1])
plt.ylabel('T12')
plt.xlabel('t')
plt.subplot(813)
plt.plot(ts, out[:, 2])
plt.ylabel('T21')
plt.xlabel('t')
plt.subplot(814)
plt.plot(ts, out[:, 3])
plt.ylabel('T22')
plt.xlabel('t')
plt.subplot(815)
plt.plot(ts, out[:, 4])
plt.ylabel('T31')
plt.xlabel('t')
plt.subplot(816)
plt.plot(ts, out[:, 5])
plt.ylabel('T32')
plt.xlabel('t')
plt.subplot(817)
plt.plot(ts, out[:, 6])
plt.ylabel('T41')
plt.xlabel('t')
plt.subplot(818)
plt.plot(ts, out[:, 7])
plt.ylabel('T42')
plt.xlabel('t')

error = []
for x, y, t, u in zip(Fx, Fy, tau, out):
    tmp = np.array([x, y, t]) - np.matmul(A_, u)
    error.append(tmp)
error = np.array(error)
# plot the error on 3x1 plot
plt.figure(3)
plt.suptitle("Error")
plt.subplot(311)
plt.plot(ts, error[:, 0])
plt.ylabel('error_Fx')
plt.xlabel('t')
plt.subplot(312)
plt.plot(ts, error[:, 1])
plt.ylabel('error_Fy')
plt.xlabel('t')
plt.subplot(313)
plt.plot(ts, error[:, 2])
plt.ylabel('error_tau')
plt.xlabel('t')

plt.show()
