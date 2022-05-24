from ament_index_python import get_package_share_directory

import opengen as og
import casadi.casadi as cs
import os
from parameters import *

b = cs.SX.sym('b', ni)
x = cs.SX.sym('x', nt)

# Bound control outputs
umax = [force] * nt
umin = [0] * nt
bounds = og.constraints.Rectangle(umin, umax)


problem = cs.transpose(x) @ (0.1*cs.SX_eye(nt)) @ x
constraint = A @ x - b


problem = og.builder.Problem(x, b, problem)     \
    .with_penalty_constraints(
        constraint
    )                                           \
    .with_constraints(bounds)                   \


meta = og.config.OptimizerMeta()        \
    .with_version('1.0.0')              \
    .with_authors(['Frank Fourlas'])    \
    .with_licence('MIT')                \
    .with_optimizer_name('tsl_optimizer')
    
    
build_config = og.config.BuildConfiguration()       \
    .with_build_directory(os.path.join(get_package_share_directory('slider_experiment'), 'python_build'))           \
    .with_build_mode('release')                     \
    .with_build_python_bindings()                   \
    .with_rebuild(False)                            \


solver_config = og.config.SolverConfiguration()     \
    .with_lbfgs_memory(32)                         \
    .with_tolerance(1e-6)                           \
    .with_max_inner_iterations(128)


builder = og.builder.OpEnOptimizerBuilder(problem,
        metadata=meta,
        build_configuration=build_config,
        solver_configuration=solver_config
)                                                   \
    .with_verbosity_level(1)


builder.build()