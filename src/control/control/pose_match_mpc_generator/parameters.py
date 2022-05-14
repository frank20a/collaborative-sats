import casadi as cs
from control.parameters import force, torque, m, nx, nu, Icm

dt = 1.0/10
nc = 1              # Number of chasers
mpc_horizon = 15    # Number of MPC steps