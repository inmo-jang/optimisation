import casadi.casadi as cs
import opengen as og

u = cs.SX.sym("u", 5)                 # decision variable (nu = 5)
p = cs.SX.sym("p", 2)                 # parameter (np = 2)
phi = og.functions.rosenbrock(u, p)   # cost function


ball = og.constraints.Ball2(None, 1.5)  # ball centered at origin
rect = og.constraints.Rectangle(xmin=[-1,-2,-3], xmax=[0, 10, -1])

# Segments: [0, 1], [2, 3, 4]
segment_ids = [1, 4]
dim = 5
bounds = og.constraints.CartesianProduct(dim, segment_ids, [ball, rect])


problem = og.builder.Problem(u, p, phi)  \
        .with_constraints(bounds)

meta = og.config.OptimizerMeta()                \
    .with_version("0.0.0")                      \
    .with_authors(["P. Sopasakis", "E. Fresk"]) \
    .with_licence("CC4.0-By")                   \
    .with_optimizer_name("the_optimizer")        

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