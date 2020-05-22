# OpEn Rust Examples

- All the subfolders are examples using OpEn Rust interface for various optimisation problems. Each of these folders is an individual rust project.

- Thus, You need to know [how to use it](https://github.com/inmo-jang/optimisation_tutorial/blob/master/tools_examples/OpEn/examples_rust/How_to_use_rust.md)

- Further details for some examples are described in the following Jupyter notebooks.

- If you want to solve 
  - A simple optimisation problem with constraints for decision variables. Then, it is better to use `PANOCOptimizer`. See examples 01 and 02. 
  - A complicated problem with $F(x) \in C$ constraints: Then, you should use `AlmOptimizer`. See the remaining examples.

## Examples

01) Rosenbrock function: [[Jupyter notebook](https://github.com/inmo-jang/optimisation_tutorial/blob/master/tools_examples/OpEn/examples_rust/OpEn_Rust_example_01_02.ipynb)] [[source](https://github.com/inmo-jang/optimisation_tutorial/tree/master/tools_examples/OpEn/examples_rust/example_01_rosenbrock)]

02) $\min (x-p_1)^2$, subject to $x < p_2$: [[Jupyter notebook](https://github.com/inmo-jang/optimisation_tutorial/blob/master/tools_examples/OpEn/examples_rust/OpEn_Rust_example_01_02.ipynb)] [[source](https://github.com/inmo-jang/optimisation_tutorial/tree/master/tools_examples/OpEn/examples_rust/example_02)]

03) ALM/PM example (for generic use of OpEn) [[Jupyter notebook](https://github.com/inmo-jang/optimisation_tutorial/blob/master/tools_examples/OpEn/examples_rust/OpEn_Rust_example_03_ALMPM.ipynb)] [[source](https://github.com/inmo-jang/optimisation_tutorial/tree/master/tools_examples/OpEn/examples_rust/example_03_almpm)]

04) $\min f(x)$ subject to $F(x) = 0$ using ALM/PM: [[Jupyter notebook](https://github.com/inmo-jang/optimisation_tutorial/blob/master/tools_examples/OpEn/examples_rust/OpEn_Rust_example_04.ipynb)] [[source](https://github.com/inmo-jang/optimisation_tutorial/tree/master/tools_examples/OpEn/examples_rust/example_04)]

05) Alternative approach for Example 04, and lessons learnt: [[Jupyter notebook](https://github.com/inmo-jang/optimisation_tutorial/blob/master/tools_examples/OpEn/examples_rust/OpEn_Rust_example_05.ipynb)]

06) Obstacle Avoidance Example (Simplified): [[Jupyter notebook](https://github.com/inmo-jang/optimisation_tutorial/blob/master/tools_examples/OpEn/examples_rust/OpEn_Rust_examples_obs_avoidance_simplified.ipynb)] [[Examples Plot Visualisation](https://github.com/inmo-jang/optimisation_tutorial/blob/master/tools_examples/OpEn/examples_rust/OpEn_Rust_examples_obs_avoidance_plotcheck.ipynb)]

07) Using a generic method for gradients, based on Example 06: [[Jupyter notebook](https://github.com/inmo-jang/optimisation_tutorial/blob/master/tools_examples/OpEn/examples_rust/OpEn_Rust_examples_general_diff.ipynb)]

08) 2D path planner avoiding an obstacle [[Jupyter notebook](https://github.com/inmo-jang/optimisation_tutorial/blob/master/tools_examples/OpEn/examples_rust/OpEn_Rust_examples_path_planner.ipynb)] [[source](https://github.com/inmo-jang/optimisation_tutorial/tree/master/tools_examples/OpEn/examples_rust/example_08_pathplanning)]

09) Avoiding multiple obstacles, based on Example 08: [[Jupyter notebook](https://github.com/inmo-jang/optimisation_tutorial/blob/master/tools_examples/OpEn/examples_rust/OpEn_Rust_examples_multiple_obstacles.ipynb)] [[source](https://github.com/inmo-jang/optimisation_tutorial/tree/master/tools_examples/OpEn/examples_rust/example_09_multple_obstacles)] 

10) Nonlinear-shaped obstacles - Part 1 (Mathematical formulations): [[Jupyter notebook](https://github.com/inmo-jang/optimisation_tutorial/blob/master/tools_examples/OpEn/examples_rust/OpEn_Rust_examples_nonlinear_obstacles.ipynb)] [[source](https://github.com/inmo-jang/optimisation_tutorial/tree/master/tools_examples/OpEn/examples_rust/example_10_nonlinear_obstacles)]

11) Nonlinear-shaped obstacles - Part 2 (Path planner that avoids multiple nonlinear-shaped obstacles): [[Jupyter notebook](https://github.com/inmo-jang/optimisation_tutorial/blob/master/tools_examples/OpEn/examples_rust/OpEn_Rust_examples_nonlinear_obstacles_02.ipynb)] [[source](https://github.com/inmo-jang/optimisation_tutorial/tree/master/tools_examples/OpEn/examples_rust/example_11_nonlinear_obstacles)]
## Etc.



- [Using Rust with Jupyter Notebook](https://github.com/google/evcxr/tree/master/evcxr_jupyter) 

- You can play with rust in [Rust Online Executer](https://play.rust-lang.org/) 

- If you are not familar with Rust, then [this YouTube tutorial](https://www.youtube.com/watch?v=vOMJlQ5B-M0&list=PLVvjrrRCBy2JSHf9tGxGKJ-bYAN_uDCUL) is worth having a look

- [rosrust: using Rust with ROS](https://github.com/adnanademovic/rosrust)
