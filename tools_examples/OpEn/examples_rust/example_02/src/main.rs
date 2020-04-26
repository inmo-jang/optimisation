//! # PANOC Example 1
//!
//! This example shows how to minimize the [Rosenbrock function] subject to constraints.
//!
//! [Rosenbrock function]: https://en.wikipedia.org/wiki/Rosenbrock_function

use optimization_engine::{constraints::*, panoc::*, *};

fn example_two_cost(p: &[f64], u: &[f64]) -> f64 {
    (u[0] - p[0]).powi(2)
}

fn example_two_grad(p: &[f64], u: &[f64], grad: &mut [f64]) {
    grad[0] = 2.0 * (u[0] - p[0]);
}

fn main_example_two(_p: &[f64]) {
    /* USER PARAMETERS */
    let tolerance = 1e-14;
    let p = _p;
    let problem_size = 2;
    let lbfgs_memory_size = 10;
    let max_iters = 80;
    let mut u = [-1.5, 0.9];
    
    let mut upper_bound: Vec<f64> = Vec::new();
    upper_bound.push(p[1]);
    // upper_bound.push(0.0);

    // define the cost function and its gradient
    let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
        example_two_grad(&p, u, grad);
        Ok(())
    };

    let f = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
        *c = example_two_cost(&p, u);
        Ok(())
    };

    // define the constraints
    let bounds = Rectangle::new(None, Option::from(upper_bound.as_slice()));
    //let bounds = Ball2::new(None, upper_bound);

    /* PROBLEM STATEMENT */
    let problem = Problem::new(&bounds, df, f);
    let mut panoc_cache = PANOCCache::new(problem_size, tolerance, lbfgs_memory_size);
    let mut panoc = PANOCOptimizer::new(problem, &mut panoc_cache).with_max_iter(max_iters);

    // Invoke the solver
    let status = panoc.solve(&mut u);

    println!("Panoc status: {:#?}", status);
    println!("Panoc solution: {:#?}", u);
}

fn main() {

    // Case 1: p_1 = 10, p_2 = 5
    main_example_two(&[10.0,5.0]);

    // Case 2: p_1 = -10, p_2 = 5
    main_example_two(&[-10.0,5.0]);
}