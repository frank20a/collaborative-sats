from ament_index_python import get_package_share_directory
from tf_utils_symbolic import quaternion_multiply, quaternion_norm, euler_from_quaternion

import opengen as og
import casadi.casadi as cs
import os

from parameters import *


def dynamics_ct(x, u):
    dx = cs.SX(nx, 1)

    # dx0 = x,        dx1 = y,        dx2 = z,
    # dx3 = x_dot,    dx4 = y_dot,    dx5 = z_dot,
    # dx6 = qx,       dx7 = qy,       dx8 = qz,       dx9 = qw,
    # dx10= wx,       dx11= wy,       dx12= wz

    # u0 = Fx,        u1 = Fy,        u2 = Fz
    # u3 = Tx,        u4 = Ty,        u5 = Tz

    # Position
    dx[0] = x[3]
    dx[1] = x[4]
    dx[2] = x[5]

    # Velocity
    dx[3:6] = u[0:3] / m

    # Quaternion
    dx[6:10] = quaternion_multiply(
        [x[10], x[11], x[12], 0],
        x[6:10]
    ) / -2.0

    # Omega
    dx[10:13] = cs.inv(Icm) @ u[3:6] - cs.inv(Icm) @ cs.cross(x[10:13], Icm @ x[10:13])

    return dx


def dynamics_dt(x, u):
    dx = dynamics_ct(x, u)

    x_ = cs.SX(nx, 1)
    for i in range(nx):
        x_[i] = x[i] + dt * dx[i]

    x_[6:10] = x_[6:10] / quaternion_norm(x_[6:10])

    return x_


def stage_cost(x, u):
    cost = 0

    for j in range(nx):
        if j in [6, 7, 8, 9]: continue
        cost += mpc_state_weights[j] * (x[j]**2)

    eul = euler_from_quaternion(x[6:10])
    cost += mpc_state_weights[6] * eul[0] + mpc_state_weights[7] * eul[1] + mpc_state_weights[8] * eul[2]

    # for j in range(nu):
    #     cost += mpc_input_weights[j] * (u[j]**2)

    return cost


def final_cost(x):
    cost = 0

    for j in range(nx):
        cost += mpc_final_weights[j] * (x[j]**2)

    return cost


u = cs.SX.sym('u', nu * mpc_horizon)   # Control Output Fx, Fy, Fz, Tx, Ty, Tz
x0 = cs.SX.sym('p', nx)                    # Parameters are model state [x, y, z, roll, pitch, yaw], their velocity and acceleration

x_t = x0
cost = 0
for i in range(mpc_horizon):
    u_t = u[i * nu: (i+1) * nu]
    
    # Update cost function
    cost += stage_cost(x_t, u_t)
    
    # Move to next time step
    x_t = dynamics_dt(x_t, u_t)

# cost += final_cost(x_t)

# Bound control outputs
umax = [force, force, force, torque, torque, torque] * mpc_horizon
umin = [-force, -force, -force, -torque, -torque, -torque] * mpc_horizon
bounds = og.constraints.Rectangle(umin, umax)


problem = og.builder.Problem(u, x0, cost)     \
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
    .with_lbfgs_memory(64)                          \
    .with_tolerance(1e-5)                           \
    .with_max_inner_iterations(256)                 \


builder = og.builder.OpEnOptimizerBuilder(problem,
    metadata=meta,
    build_configuration=build_config,
    solver_configuration=solver_config).with_verbosity_level(1)


builder.build()