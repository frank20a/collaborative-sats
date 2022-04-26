import numpy as np
import casadi as cs

force = 2.0
torque = 5e-2
dt = 1.0/5
m = 30.0

# I = np.diag([30 * (0.25**2 + 0.05**2) / 12, 30 * (0.25**2 + 0.05**2) / 12, 30 * (0.25**2 + 0.25**2) / 12])
I_ = cs.SX(3, 3)
for i in range(3): 
    I_[i, i] = [30 * (0.25**2 + 0.05**2) / 12, 30 * (0.25**2 + 0.05**2) / 12, 30 * (0.25**2 + 0.25**2) / 12][i]

# A = np.zeros((9, 9), dtype=np.float32)
# A[0:6, 3:9] = np.eye(6, dtype=np.float32) * dt
# A[0:3, 6:9] = np.eye(3, dtype=np.float32) * dt * dt / 2
# A[0:3, 0:3] = np.eye(3, dtype=np.float32)
# A[3:6, 3:6] = np.eye(3, dtype=np.float32)
# A = np.concatenate((
#     np.concatenate((A, np.zeros((9, 9))), axis=1), 
#     np.concatenate((np.zeros((9, 9)), A), axis=1)), 
#     axis=0
# )
A_ = cs.SX(9, 9)
A_[0:6, 3:9] = cs.SX.eye(6) * dt
A_[0:3, 6:9] = cs.SX.eye(3) * dt * dt / 2
A_[0:3, 0:3] = cs.SX.eye(3)
A_[3:6, 3:6] = cs.SX.eye(3)
A_ = cs.vertcat(
    cs.horzcat(A_, cs.SX(9, 9)),
    cs.horzcat(cs.SX(9, 9), A_)
)

# B = np.zeros((9, 3), dtype=np.float32)
# B[6:9, 0:3] = np.eye(3, dtype=np.float32) / m
# B_ = B
# B_[6:9, 0:3] = I
# B = np.concatenate((
#     np.concatenate((B, np.zeros((9, 3))), axis=1), 
#     np.concatenate((np.zeros((9, 3)), B_), axis=1)), 
#     axis=0
# )
B_ = cs.SX(18, 6)
B_[6:9, 0:3] = cs.SX.eye(3) / m
B_[15:18, 3:6] = cs.inv(I_)

# C = np.zeros((6, 18))
# C[0:3,  0:3] = np.eye(3, dtype=np.float32)
# C[3:6, 9:12] = np.eye(3, dtype=np.float32)
C_ = cs.SX(6, 18)
C_[0:3,  0:3] = cs.SX.eye(3)
C_[3:6, 9:12] = cs.SX.eye(3)

# D = np.zeros((6, 6))
D_ = cs.SX(6, 6)

mpc_horizon = 30
control_dim = 6
n_states = 18
mpc_cost_weights = [1e14, 1e14, 1e14, 5e8, 5e8, 5e8]
# mpc_cost_weights = [1e3, 1e3, 1e3, 5, 5, 5]