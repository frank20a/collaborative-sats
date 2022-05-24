import casadi as cs
from control.parameters import nx, force, torque

m = 4.528
Icm = cs.SX(3, 3)
for i in range(3): 
    Icm[i, i] = 0.109214481
dt = 1.0/10
nc = 1              # Number of chasers
nu = 3              # Number of controls
mpc_horizon = 30    # Number of MPC steps