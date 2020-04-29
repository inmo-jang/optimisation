//! # Augmented Lagrangian and Penalty Method
//!
//! This example shows how to solve an optimization problem based on the [Rosenbrock function].
//!
//! [Rosenbrock function]: https://en.wikipedia.org/wiki/Rosenbrock_function

use optimization_engine::{
    alm::*,
    core::{constraints::*, panoc::*},
    matrix_operations, SolverError,
};

// Smooth cost function
pub fn f(u: &[f64], cost: &mut f64) -> Result<(), SolverError> {
        *cost = u[2];
        Ok(())
}

pub fn df(u: &[f64], grad: &mut [f64]) -> Result<(), SolverError> {
        grad[0] = 0.0;
        grad[1] = 0.0;
        grad[2] = 1.0;    
        Ok(())
}

pub fn f1(u: &[f64], f1u: &mut [f64]) -> Result<(), SolverError> {
    f1u[0] = 2.0*u[0] - u[2] + 1.0;
    f1u[1] = 0.5 * (u[0].powi(2)+u[1].powi(2)) - u[2];
    Ok(())
}

pub fn f1_jacobian_product(u: &[f64], d: &[f64], res: &mut [f64]) -> Result<(), SolverError> {
    res[0] = 2.0*d[0] + u[0]*d[1];
    res[1] = u[1]*d[1];
    res[2] = -d[0] - d[1];
    Ok(())
}

fn main() {
    let tolerance = 1e-5;
    let nx = 3; // problem_size: dimension of the decision variables
    let n1 = 2; // range dimensions of mappings F1
    let n2 = 0; // range dimensions of mappings F2
    let lbfgs_mem = 5; // memory of the LBFGS buffer
    
    // PANOCCache: All the information needed at every step of the algorithm
    let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
    
    // AlmCache: A cache structure that contains all the data 
    // that make up the state of the ALM/PM algorithm
    // (i.e., all those data that the algorithm updates)
    let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);

    let set_c = Zero::new(); // Set C
    let bounds = Ball2::new(None, 100.0); // Set U
    let set_y = Ball2::new(None, 1e12);  // Set Y

    // AlmFactory: Prepare function psi and its gradient 
    // given the problem data such as f, del_f and 
    // optionally F_1, JF_1, C, F_2
    let factory = AlmFactory::new(
        f, // Cost function
        df, // Cost Gradient
        Some(f1), // MappingF1
        Some(f1_jacobian_product), // Jacobian Mapping F1 Trans
        NO_MAPPING, // MappingF2
        NO_JACOBIAN_MAPPING, // Jacobian Mapping F2 Trans
        Some(set_c), // Constraint set
        n2,
    );

    // Define an optimisation problem 
    // to be solved with AlmOptimizer
    let alm_problem = AlmProblem::new(
        bounds,
        Some(set_c),
        Some(set_y),
        |u: &[f64], xi: &[f64], cost: &mut f64| -> Result<(), SolverError> {
            factory.psi(u, xi, cost)
        },
        |u: &[f64], xi: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
            factory.d_psi(u, xi, grad)
        },
        Some(f1),
        NO_MAPPING,
        n1,
        n2,
    );

    let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
        .with_delta_tolerance(1e-5)
        .with_max_outer_iterations(200)
        .with_epsilon_tolerance(1e-6)
        .with_initial_inner_tolerance(1e-2)
        .with_inner_tolerance_update_factor(0.5)
        .with_initial_penalty(100.0)
        .with_penalty_update_factor(1.05)
        .with_sufficient_decrease_coefficient(0.2)
        .with_initial_lagrange_multipliers(&vec![5.0; n1]);

    let mut u = vec![0.0; nx];
    let solver_result = alm_optimizer.solve(&mut u);
    let r = solver_result.unwrap();
    println!("\n\nSolver result : {:#.7?}\n", r);
    println!("Solution u = {:#.6?}", u);
}