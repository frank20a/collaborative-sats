from ament_index_python import get_package_share_directory
from control.tf_utils_symbolic import quaternion_multiply, quaternion_norm, euler_from_quaternion, quaternion_inverse, vector_rotate_quaternion

import opengen as og
import casadi.casadi as cs
import os

from parameters import force, torque, dt, m, Icm, nc, nu, nx, mpc_horizon


def chaser_dynamics_ct(x, u):
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
    dx[3:6] = vector_rotate_quaternion(u[0:3], x[6:10]) / m

    # Quaternion
    dx[6:10] = quaternion_multiply(
        [x[10], x[11], x[12], 0],
        x[6:10],
    ) / 2.0

    # Omega
    dx[10:13] = cs.inv(Icm) @ vector_rotate_quaternion(u[3:6], x[6:10]) - cs.inv(Icm) @ cs.cross(x[10:13], Icm @ x[10:13])

    return dx


def chaser_dynamics_dt(x, u):
    dx = chaser_dynamics_ct(x, u)

    x_ = cs.SX(nx, 1)
    for i in range(nx):
        x_[i] = x[i] + dt * dx[i]

    x_[6:10] = x_[6:10] / quaternion_norm(x_[6:10])

    return x_


def target_dynamics_ct(x):
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
    dx[3:6] = 0

    # Quaternion
    dx[6:10] = quaternion_multiply(
        [x[10], x[11], x[12], 0],
        x[6:10],
    ) / 2.0

    # Omega
    dx[10:13] = 0

    return dx


def target_dynamics_dt(x):
    dx = target_dynamics_ct(x)

    x_ = cs.SX(nx, 1)
    for i in range(nx):
        x_[i] = x[i] + dt * dx[i]

    x_[6:10] = x_[6:10]

    return x_


def sigmoid(x):
    return 2 * (cs.exp(x) / (1 + cs.exp(x)) - 0.5)


def stage_cost(xs, ref, offset, us, Qs, Qi):
    cost = 0

    for n, (x, u, off) in enumerate(zip(xs, us, offset)):
        # ======== State cost ========
        # position
        tmp = vector_rotate_quaternion(ref[0:3] + off[0:3], ref[6:10])
        for i in range(3): cost += Qs[0] * ((x[i] - tmp[i])**2)

        # velocity
        for i in range(3, 6): cost += Qs[1] * ((x[i] - ref[i])**2)

        # orientation
        # eul = euler_from_quaternion(quaternion_multiply(
        #     x[6:10],
        #     quaternion_inverse(ref[6:10]),
        # ))
        # tmp = off[3:6]
        # for i in range(3): cost += Qs[2] * sigmoid((eul[i] - tmp[i])**2)
        # q_err = quaternion_multiply(
        #     off[3:7],
        #     ref[6:10],
        # )
        tmp = quaternion_multiply(
            off[3:7],
            ref[6:10],
        )
        q_err = quaternion_multiply(
            x[6:10],
            quaternion_inverse(tmp),
        )
        q_err /= quaternion_norm(q_err)
        for i in range(3): cost += Qs[2] * q_err[i]**2

        # omega
        for i in range(10, 13): cost += Qs[3] * ((x[i] - ref[i])**2)

        # ======== Input cost ========
        for i in range(nu): cost += Qi[i // 3] * (u[i]**2)

    return cost


def final_cost(x, ref, offset, Qf):
    cost = stage_cost(x, ref, offset, [[0] * nu for i in range(nc)], Qf, [0, 0])

    return cost


u = cs.SX.sym('u', nu * nc * mpc_horizon)
p = cs.SX.sym('p', nx * (nc + 1) + 7*nc + 10)
x0 = [p[nx * i:nx * (i+1)] for i in range(nc)]
x0_tar = p[nx * nc: nx * (nc + 1)]
offset = [p[nx * (nc + 1) + i*7: nx * (nc + 1) + (i+1)*7] for i in range(nc)]
state_weights = p[nx * (nc + 1) + 7*nc + + 0 : nx * (nc + 1) + 7*nc + + 4]
input_weights = p[nx * (nc + 1) + 7*nc + + 4 : nx * (nc + 1) + 7*nc + + 6]
final_weights = p[nx * (nc + 1) + 7*nc + + 6 : nx * (nc + 1) + 7*nc + + 10]

xt = x0
xt_tar = x0_tar
cost = 0
for i in range(mpc_horizon):
    ut = [u[i*nu*nc + j*nu: i*nu*nc + (j+1)*nu] for j in range(nc)]
    
    # Update cost function
    cost += stage_cost(xt, xt_tar, offset, ut, state_weights, input_weights)
    
    # Move to next time step
    for j in range(nc):
        xt[j] = chaser_dynamics_dt(xt[j], ut[j])
    xt_tar = target_dynamics_dt(xt_tar)
cost += final_cost(xt, xt_tar, offset, final_weights)

# Bound control outputs
umax = [force, force, force, torque, torque, torque] * mpc_horizon
umin = [-force, -force, -force, -torque, -torque, -torque] * mpc_horizon
bounds = og.constraints.Rectangle(umin, umax)


problem = og.builder.Problem(u, p, cost)     \
    .with_constraints(bounds)


meta = og.config.OptimizerMeta()        \
    .with_version('0.2.0')              \
    .with_authors(['Frank Fourlas'])    \
    .with_licence('MIT')                \
    .with_optimizer_name('pose_match_mpc')
    
    
build_config = og.config.BuildConfiguration()       \
    .with_build_directory(os.path.join(get_package_share_directory('control'), 'python_build'))           \
    .with_build_mode('release')                     \
    .with_build_python_bindings()                   \
    .with_rebuild(False)                            \
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