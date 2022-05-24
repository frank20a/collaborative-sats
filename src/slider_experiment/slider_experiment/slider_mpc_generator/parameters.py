import casadi as cs

force = 1.2
torque = 0.54

m = 4.528
Icm = cs.SX(3, 3)
for i in range(3): 
    Icm[i, i] = 0.109214481
dt = 1.0/10
nc = 1              # Number of chasers
nu = 3              # Number of controls
nx = 13             # Number of states
mpc_horizon = 30    # Number of MPC steps