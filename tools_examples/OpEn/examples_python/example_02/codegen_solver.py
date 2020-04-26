import casadi.casadi as cs
import opengen as og
import numpy as np

x = cs.SX.sym("x", 1)                 # decision variable (num_x = 1)
p = cs.SX.sym("p", 2)                 # parameter (num_p = 2)
f = (x[0]-p[0])**2                    # cost function



f2 = cs.vertcat( cs.fmax(0.0, x[0]-p[1]) )

problem = og.builder.Problem(x, p, f)  \
        .with_penalty_constraints(f2)  \


meta = og.config.OptimizerMeta()                \
    .with_version("0.0.0")                      \
    .with_licence("CC4.0-By")                   \
    .with_optimizer_name("example_02")        

build_config = og.config.BuildConfiguration()  \
    .with_build_directory("python_build")      \
    .with_build_mode("debug")                  \
    .with_tcp_interface_config()    

solver_config = og.config.SolverConfiguration()   \
            .with_lfbgs_memory(15)                \
            .with_tolerance(1e-5)                 \
            .with_max_inner_iterations(155)

builder = og.builder.OpEnOptimizerBuilder(problem,
                                          metadata=meta,
                                          build_configuration=build_config,
                                          solver_configuration=solver_config)
builder.build()                