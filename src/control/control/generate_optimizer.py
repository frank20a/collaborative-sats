from ament_index_python import get_package_share_directory

import opengen as og
import casadi.casadi as cs
import os

from parameters import *


u = cs.SX.sym('u', 6)
p = cs.SX.sym('p', 12)
phi = og.functions.norm2(p[0:6] - p[6:12])


valsF = og.constraints.FiniteSet([[-force], [0], [force]])
valsT = og.constraints.FiniteSet([[-torque], [0], [torque]])
bounds = og.constraints.CartesianProduct(range(6), [valsF, valsF, valsF, valsT, valsT, valsT])


problem = og.builder.Problem(u, p, phi)     \
    .with_constraints(bounds)


meta = og.config.OptimizerMeta()        \
    .with_version('0.1.0')              \
    .with_authors(['Frank Fourlas'])    \
    .with_licence('MIT')                \
    .with_optimizer_name('chaser_mpc')
    
    
build_config = og.config.BuildConfiguration()       \
    .with_build_directory(os.path.join(get_package_share_directory('control'), 'python_build'))           \
    .with_build_mode('debug')                       \
    .with_build_python_bindings()
    # .with_tcp_interface_config()                    # IP 127.0.0.1 and port 8333


solver_config = og.config.SolverConfiguration()     \
    .with_lbfgs_memory(15)                          \
    .with_tolerance(1e-5)                           \
    .with_max_inner_iterations(155)


builder = og.builder.OpEnOptimizerBuilder(problem,
    metadata=meta,
    build_configuration=build_config,
    solver_configuration=solver_config)


builder.build()