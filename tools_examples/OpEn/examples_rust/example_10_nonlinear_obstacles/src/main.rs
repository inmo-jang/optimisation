// Nonlinear-shaped Obstacle Avoidance Path Planning Example with Multiple Obstacles
// by Dr Inmo Jang (inmo3592@gmail.com) 

use optimization_engine::{
    alm::*,
    constraints::*, panoc::*, *
};
use nalgebra::base::{*};
// For plot
use plotlib::page::Page;
use plotlib::repr::Plot;
use plotlib::view::ContinuousView;
use plotlib::style::{PointMarker, PointStyle};


// Obstacles
// ------------ (1) Elipsoid ------------ 
fn h_elip(x: &[f64], elip: (f64, f64, f64, f64), print: bool) -> f64{
    // x: user position
    // elip : (centre_x, centre_y, radius_x, radius_y)
    // print: Flag to indicate whether to print the result

    let h = (1.0 - ((x[0]-elip.0)/elip.2).powi(2) - ((x[1]-elip.1)/elip.3).powi(2) ).max(0.0);  
    
    if print {
        if h > 0.0 {
            println!("x is inside the obstacle");
        }
        else{
            println!("x is outside the obstacle");
        }
    }    
    return h;
}


// Points for Elipsoid (For Plot)
fn get_points_elip(elip: (f64, f64, f64, f64), search_area: ((f64, f64), (f64, f64), f64) ) -> Vec<(f64, f64)> {
    let mut points = Vec::new(); // Output initialisation    
    let x_range = search_area.0;
    let y_range = search_area.1;
    let resolution = search_area.2;
    
    let mut x = x_range.0; // Start point
    let mut y = y_range.0;

    while x <= x_range.1{
        while y <= y_range.1{            
            let h = h_elip(&[x,y], elip.clone(), false);
            if h > 0.0 {
                points.push((x,y));
            } 
            y += resolution;
        }
        x += resolution;
        y = y_range.0;
    }
    return points;
}

// ------------  (2) Polyhedral ------------ 
fn h_poly(x: &[f64], poly: Vec<((f64, f64), f64)>, centre: (f64, f64), print: bool) -> f64{
    // x: user position
    // poly : <(a_i, b_i)>    
    // print: Flag to indicate whether to print the result

    let mut h: f64 = 1.0;
    for i in 0..poly.len(){
        let a_i = poly[i].0;
        let b_i = poly[i].1;
        h *= (b_i - (a_i.0*(x[0] - centre.0) + a_i.1*(x[1]- centre.1))).max(0.0);        
    }
    
    if print {
        if h > 0.0 {
            println!("x is inside the obstacle");
        }
        else{
            println!("x is outside the obstacle");
        }
    } 
    
    return h;
}

// Points for Polyhedral (For Plot)
fn get_points_poly(poly: Vec<((f64, f64), f64)>, centre: (f64, f64), search_area: ((f64, f64), (f64, f64), f64)) -> Vec<(f64, f64)> {
    let mut points = Vec::new(); // Output initialisation    
    let x_range = search_area.0;
    let y_range = search_area.1;
    let resolution = search_area.2;
    
    let mut x = x_range.0; // Start point
    let mut y = y_range.0;

    while x <= x_range.1{
        while y <= y_range.1{            
            let h = h_poly(&[x,y], poly.clone(), centre.clone(), false);
            if h > 0.0 {
                points.push((x,y));
            } 
            y += resolution;
        }
        x += resolution;
        y = y_range.0;
    }
    return points;
}

// ------------  (3) Nonlinear Obstacles in Sathya et al. 2019 ------------
fn h_nlr_1(x: &[f64], centre: (f64, f64), print: bool) -> f64{
    // x: user position
    // centre : (c_x, c_y)    
    
    let h1 = ((x[1] - centre.1) - ((x[0] - centre.0)).powi(2)).max(0.0);
    let h2 = (1.0 + (x[0] - centre.0).powi(2)/2.0 - (x[1] - centre.1)).max(0.0);
    
    let h = h1*h2;
    
    if print {
        if h > 0.0 {
            println!("x is inside the obstacle");
        }
        else{
            println!("x is outside the obstacle");
        }
    } 
    
    return h;
}



fn h_nlr_2(x: &[f64], centre: (f64, f64), print: bool) -> f64{
    // x: user position
    // centre : (c_x, c_y)    
    
    let h1 = ((x[1] - centre.1) - 2.0*(-(x[0] - centre.0)/2.0).sin()).max(0.0);
    let h2_1 = (3.0*(((x[0] - centre.0)/2.0) - 1.0).sin() - (x[1] - centre.1)).max(0.0);
    let h2_2 = (x[0] - centre.0 - 1.0).max(0.0);
    let h2_3 = (8.0 - (x[0] - centre.0)).max(0.0);

        
    let h = h1*h2_1*h2_2*h2_3;
    
    if print {
        if h > 0.0 {
            println!("x is inside the obstacle");
        }
        else{
            println!("x is outside the obstacle");
        }
    } 
    
    return h;
}

// Points for Predefined object (For Plot)
fn get_points_predefined_obj(obj_kind: u32, centre: (f64, f64), search_area: ((f64, f64), (f64, f64), f64)) -> Vec<(f64, f64)> {
    // obj_kind: object kind

    let mut points = Vec::new(); // Output initialisation    
    let x_range = search_area.0;
    let y_range = search_area.1;
    let resolution = search_area.2;
    
    let mut x = x_range.0; // Start point
    let mut y = y_range.0;
    

    while x <= x_range.1{
        while y <= y_range.1{
            let mut h = 0.0;
            match obj_kind {
                1 => h = h_nlr_1(&[x,y], centre.clone(), false), 
                2 => h = h_nlr_2(&[x,y], centre.clone(), false), 
                _ => h=0.0,
            }            
            
            if h > 0.0 {
                points.push((x,y));
            }; 
            y += resolution;
        }
        x += resolution;
        y = y_range.0;
    }
    return points;
}



fn main(){
    let elip_A = (5.0, 0.0, 2.0, 1.5);
    let mut poly_A = Vec::new();
    poly_A.push(((1.0, 0.0), 1.0));
    poly_A.push(((-1.0, 0.0), 1.0));
    poly_A.push(((0.0, 1.0), 1.0));
    poly_A.push(((0.0, -1.0), 1.0));

    let mut poly_B = Vec::new();
    poly_B.push(((1.0, 2.0), 2.0));
    poly_B.push(((-2.0, 1.0), 2.0));
    poly_B.push(((-1.0, -1.0), 2.0));
    poly_B.push(((1.0, -1.0), 2.0));
    poly_B.push(((0.0, -1.0), 1.5));


    let search_area = ((-10.0, 10.0), (-10.0, 10.0), 0.05);
    let p_elip_A = get_points_elip(elip_A.clone(), search_area.clone());
    let p_poly_A = get_points_poly(poly_A.clone(), (0.0, 0.0), search_area.clone());
    let p_poly_B = get_points_poly(poly_B.clone(), (0.0, 5.0), search_area.clone());

    let p_nlr_1 = get_points_predefined_obj(1, (-5.0, 0.0), search_area.clone());
    let p_nlr_2 = get_points_predefined_obj(2, (-5.0, -5.0), search_area.clone());

    // ======================= (2) Plot ================================
    let s1: Plot = Plot::new(p_elip_A.clone()).point_style(
        PointStyle::new()
            // .marker(PointMarker::Square) // setting the marker to be a square
            .size(0.5)
            .colour("#DD3355"),
    ); 

    let s2: Plot = Plot::new(p_poly_A.clone()).point_style(
        PointStyle::new() // uses the default marker
            .size(0.5)
            .colour("#55dd33"),
    ); 

    let s3: Plot = Plot::new(p_nlr_1.clone()).point_style(
        PointStyle::new() // uses the default marker
            .size(0.5)
            .colour("#bb33dd"),
    ); 

    let s4: Plot = Plot::new(p_nlr_2.clone()).point_style(
        PointStyle::new() // uses the default marker
            .size(0.5)
            .colour("#35C788"),
    );     

    let s5: Plot = Plot::new(p_poly_B.clone()).point_style(
        PointStyle::new() // uses the default marker
            .size(0.5)
            .colour("#ddbb33"),
    ); 

    // Plot: The 'view' describes what set of data is drawn
    let v = ContinuousView::new()
        .add(s1)
        .add(s2)
        .add(s3)
        .add(s4)
        .add(s5)


        .x_range(-10., 10.)
        .y_range(-10., 10.)
        .x_label("X (m)")
        .y_label("Y (m)");

    // A page with a single view is then saved to an SVG file
    Page::single(&v).save("result.svg").unwrap();

    println!("Done - Visual Result Generated");


}

/* The following will be used for path planning 

// Problem Master 
pub struct ProblemMaster{
    x_now: Matrix2x1<f64>, // Robot Start Position
    x_ref: Matrix2x1<f64>, // Robot Goal Position
    x_obs: Vec<(f64, f64, f64)>, // Obstacle Position and Radius
    u_max: f64, // Dyanmics Radius
}

impl ProblemMaster{
    pub fn init(_x_start: Matrix2x1<f64>, _x_ref: Matrix2x1<f64>, _x_obs: Vec<(f64, f64, f64)>, _u_max: f64) -> Self {
        let x_now = _x_start;
        let x_ref = _x_ref;
        let x_obs = _x_obs;
        let u_max = _u_max;
        Self{x_now, x_ref, x_obs, u_max}            
    }
    
    
    // ========= Cost function (You need to modify this) =========
    pub fn f_call(&self, u: &[f64]) -> f64{        
        let cost = (u[0]-self.x_ref[(0,0)]).powi(2) + (u[1]-self.x_ref[(1,0)]).powi(2);
        cost
    }
    // ===========================================================
    
    pub fn f(&self, u: &[f64], cost: &mut f64){
        *cost = self.f_call(u);        
    }
    
    // Gradient of the cost function
    pub fn df(&self, u: &[f64], grad: &mut [f64]){
        let f_0 = self.f_call(u);
        
        for i in 0..u.len() {
            let mut u_h = u.to_vec();
            u_h[i] += 0.000001;
            let f_h = self.f_call(u_h.as_slice());
            grad[i] = (-f_0 + f_h) / 0.000001;
        }
       
    } 
    
    // ========= F1 Constraint (You need to modify this) =========
    pub fn f1_call(&self, u: &[f64])-> Vec<f64> {
        let mut f1u = vec![0.0; u.len()];
        // Obstacle Avoidance Constraint (C2)
        let mut f1u_0: f64 = 0.0;        
        for j in 0..self.x_obs.len(){
            f1u_0 += (1.0*(self.x_obs[j].2).powi(2) - (u[0]-self.x_obs[j].0).powi(2) - (u[1]-self.x_obs[j].1).powi(2) ).max(0.0);  
        }
        f1u[0] = f1u_0;

        // Dynamics Constraint (C1)
        let mut u_now = Matrix2x1::new(0.0, 0.0);
        for i in 0..u.len(){
            u_now[(i,0)] = u[i];
        }
        let delta = u_now - self.x_now;
        f1u[1] = (delta.norm() - self.u_max).max(0.0);
        
        return f1u;
    }
    // ===========================================================
    
    pub fn f1(&self, u: &[f64], f1u: &mut [f64]){
        let f1u_vec = self.f1_call(u); 
        for i in 0..f1u_vec.len(){
            f1u[i] = f1u_vec[i];
        }
    }    
    
    // Jacobian of F1
    pub fn jf1_call(&self, u: &[f64])-> Matrix2<f64> {
        let mut jf1 = Matrix2::new(0.0, 0.0,
                              0.0, 0.0);
        
        let f1_0 = self.f1_call(u);

        for i in 0..f1_0.len(){
            for j in 0..u.len() {
                let mut u_h = u.to_vec();
                u_h[j] += 0.000001;
                let f_h = self.f1_call(u_h.as_slice());
                jf1[(i,j)] = (-f1_0[i] + f_h[i]) / 0.000001;
            }                        
        }

        return jf1;        
    } 
    
    // Jacobian Product (JF_1^{\top}*d)
    pub fn f1_jacobian_product(&self, u: &[f64], d: &[f64], res: &mut [f64]){
        let test = self.f1_call(u);
        
        let mut jf1_matrix = self.jf1_call(u);
        if test[0] < 0.0{ // Outside the obstacle
            jf1_matrix[(0,0)] = 0.0;
            jf1_matrix[(0,1)] = 0.0;  
        }          
        
        let mut d_matrix = Matrix2x1::new(0.0, 0.0);
        for i in 0..d.len(){
            d_matrix[(i,0)] = d[i];
        }
        
        let res_matrix =  jf1_matrix.transpose()*d_matrix;
        
        res[0] = res_matrix[(0,0)];
        res[1] = res_matrix[(1,0)];  
    }
    
}

// Optimisation Loop for Each Time Step
fn main_loop(_x_now: &[f64], _x_ref: &[f64], _x_obs: Vec<(f64, f64, f64)>, _u_max: f64) -> Vec<f64> {
 
    // ===========================================
    let mut x_now = Matrix2x1::new(0.0, 0.0);
    for i in 0.._x_now.len(){
        x_now[(i,0)] = _x_now[i];
    }
    
    let mut x_ref = Matrix2x1::new(0.0, 0.0);
    for i in 0.._x_ref.len(){
        x_ref[(i,0)] = _x_ref[i];
    }
    
    let x_obs = _x_obs;
    
    let u_max = _u_max;        
    
    let pm = ProblemMaster::init(x_now, x_ref, x_obs, u_max);
    
    // ===========================================
    
    let tolerance = 1e-5;
    let nx = 2; // problem_size: dimension of the decision variables
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
    let bounds = Ball2::new(None, 1e12); // Set U
    let set_y = Ball2::new(None, 1e12);  // Set Y

    // ============= 
    // Re-define the functions linked to user parameters
    let f = |u: &[f64], cost: &mut f64| -> Result<(), SolverError> {
        pm.f(u, cost);
        Ok(())
    };
    
    let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
        pm.df(u, grad);
        Ok(())
    };
    
    let f1 = |u: &[f64], f1u: &mut [f64]| -> Result<(), SolverError> {
        pm.f1(u, f1u);
        Ok(())
    };    
    
    let f1_jacobian_product = |u: &[f64], d: &[f64], res: &mut [f64]| -> Result<(), SolverError> {
        pm.f1_jacobian_product(u,d,res);
        Ok(())
    };      
    // ==============
    
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

    let mut u = _x_now.to_vec(); // vec![0.0; nx]; // Initial guess
    let solver_result = alm_optimizer.solve(&mut u);
    let r = solver_result.unwrap();
//     println!("\n\nSolver result : {:#.7?}\n", r);
//     println!("Solution u = {:#.6?}", u);
    
    return u;
    
}

// Function for Each Instance 
fn main_instance(_x_now: &[f64], _x_ref: &[f64], _x_obs: Vec<(f64, f64, f64)>, _u_max: f64) -> Vec<(f64, f64)>  {
    
    let mut done = false;
    let mut x_now = _x_now.to_vec();
        
    let mut path_result = Vec::new();
    path_result.push((x_now[0], x_now[1]));
    println!("Solution x_now = {:#.6?}   {:#.6?}", x_now[0], x_now[1]);

    while !done{    
        let x_next = main_loop(x_now.as_slice(), _x_ref, _x_obs.clone(), _u_max);

        x_now = x_next;
        let delta = ((x_now[0]-_x_ref[0]).powi(2)+(x_now[1]-_x_ref[1]).powi(2)).sqrt();
        if delta < 0.00001{
            done = true;
        }
        path_result.push((x_now[0], x_now[1]));
        println!("Solution x_now = {:#.6?}   {:#.6?}", x_now[0], x_now[1]);   
    }
          
    return path_result;
            
        
}

// Points for Obstacle (For Plot)
fn get_points_obstacle(_x_obs: Vec<(f64, f64, f64)>) -> Vec<(f64, f64)> {
    let mut points = Vec::new(); // Output initialisation

    

    for j in 0.._x_obs.len(){
        let obs_centre = _x_obs[j];        
        for i in 0..36{
            let radius = obs_centre.2;
            let i_f64 = i as f64;
            let angle = 10.0*i_f64*std::f64::consts::PI/180.0;
    
            let x = radius*angle.cos() + obs_centre.0;
            let y = radius*angle.sin() + obs_centre.1;
    
            points.push((x,y));
            println!("Circle = {:#.6?}   {:#.6?}", x, y);
        }        
    }

    return points;

}

// Main function
fn main_main(){
    
    // Problem Instance Parameters
    let pos_start = &[0.0, 0.0];
    let pos_goal = &[10.0, 10.0];
    let mut pos_obstacle = Vec::new();
    pos_obstacle.push((5.0, 4.0, 1.5)); // (pos_x, pos_y, radius)
    pos_obstacle.push((1.0, 2.0, 1.0));
    pos_obstacle.push((8.0, 7.0, 2.0));
    let max_movement = 0.1_f64;

    // ==================== (1) Get Points =============================
    // Generate a path avoding the obstacle
    let path = main_instance(pos_start, pos_goal, pos_obstacle.clone(), max_movement);    
    // Get the points for the obstacle
    let obstacle = get_points_obstacle(pos_obstacle.clone());


    // ======================= (2) Plot ================================
    // Path Plot
    let data1 = path.clone();
    let s1: Plot = Plot::new(data1).point_style(
        PointStyle::new()
            .marker(PointMarker::Square) // setting the marker to be a square
            .size(1.0)
            .colour("#DD3355"),
    ); 

    // Obstacle Plot
    let data2 = obstacle.clone();
    let s2: Plot = Plot::new(data2).point_style(
        PointStyle::new() // uses the default marker
            .size(2.0)
            .colour("#35C788"),
    ); 

    // Starting point & Goal point
    let point_start = vec![(pos_start[0], pos_start[1])];
    let s3: Plot = Plot::new(point_start).point_style(
        PointStyle::new() // uses the default marker
            .colour("#35C788"),
    ); // and a different colour

    let point_goal = vec![(pos_goal[0], pos_goal[1])];
    // We can plot multiple data sets in the same view
    let s4: Plot = Plot::new(point_goal).point_style(
        PointStyle::new() // uses the default marker
            .colour("#35C788"),
    ); // and a different colour


    // Plot: The 'view' describes what set of data is drawn
    let v = ContinuousView::new()
        .add(s2)
        .add(s1)
        .add(s3)
        .add(s4)

        .x_range(-5., 15.)
        .y_range(-5., 15.)
        .x_label("X (m)")
        .y_label("Y (m)");

    // A page with a single view is then saved to an SVG file
    Page::single(&v).save("path_result.svg").unwrap();

    println!("Done - Visual Result Generated");
}
*/