import casadi as cs
from control.parameters import force, torque, m, nx, nu, Icm

dt = 1.0/10
nc = 4                      # Number of chasers
dur = 3                     # Horizon in seconds
mpc_horizon = int(dur/dt)   # Number of MPC steps
falloff = 0.6               # Falloff percentage