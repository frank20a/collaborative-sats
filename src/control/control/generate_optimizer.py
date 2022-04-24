from ament_index_python import get_package_share_directory
from tf_utils_symbolic import vector_rotate_quaternion, euler_from_quaternion

import opengen as og
import casadi.casadi as cs
import os

from parameters import *


u = cs.SX.sym('u', control_dim * mpc_horizon)   # Control Output Fx, Fy, Fz, Tx, Ty, Tz
p = cs.SX.sym('p', n_states)                    # Parameters are model state [x, y, z, roll, pitch, yaw], their velocity and acceleration
cost = 0

x = p
for i in range(mpc_horizon):
    # Rotate forces vector to robot orientation
    # rot_force = vector_rotate_quaternion(u[i * control_dim: i * control_dim + 3], p[9:13])
    
    # System dynamics
    # x_ = A_@x + B_@ cs.vertcat(rot_force, u[i * control_dim + 3:i * control_dim + 6])
    x_ = A_@x + B_@u[i * control_dim: (i+1) * control_dim]
    y  = C_@x + D_@u[i * control_dim: (i+1) * control_dim]
    
    # Update cost function
    for i in range(6):
        cost += mpc_cost_weights[i] * y[i]**2
    
    # Move to next time step
    x = x_


# Bound control outputs to fixed force and torque values
valsF = [og.constraints.FiniteSet([[-force], [0], [force]])] * 3
valsT = [og.constraints.FiniteSet([[-torque], [0], [torque]])] * 3
bounds = og.constraints.CartesianProduct(
    range(control_dim * mpc_horizon), 
    (valsF + valsT) * mpc_horizon
)


problem = og.builder.Problem(u, p, cost)     \
    .with_constraints(bounds)


meta = og.config.OptimizerMeta()        \
    .with_version('0.1.0')              \
    .with_authors(['Frank Fourlas'])    \
    .with_licence('MIT')                \
    .with_optimizer_name('chaser_mpc')
    
    
build_config = og.config.BuildConfiguration()       \
    .with_build_directory(os.path.join(get_package_share_directory('control'), 'python_build'))           \
    .with_build_mode('release')                     \
    .with_build_python_bindings()                   \
    .with_rebuild(False)                             \
    # .with_tcp_interface_config()                    # IP 127.0.0.1 and port 8333


solver_config = og.config.SolverConfiguration()     \
    .with_lbfgs_memory(20)                          \
    .with_tolerance(1e-6)                           \
    .with_max_inner_iterations(155)                 \
    .with_penalty_weight_update_factor(150)           \
    .with_max_outer_iterations(10)                 \
    .with_initial_penalty(10000000.0)                   \


builder = og.builder.OpEnOptimizerBuilder(problem,
    metadata=meta,
    build_configuration=build_config,
    solver_configuration=solver_config).with_verbosity_level(1)


builder.build()