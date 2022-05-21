import casadi as cs
from control.parameters import force, torque, m, nx, nu, Icm

dt = 1.0/10
nc = 2              # Number of chasers
mpc_horizon = 30    # Number of MPC steps