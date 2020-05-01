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
## Etc.

06) Obstacle Avoidance Example (Simplified): [[Jupyter notebook](https://github.com/inmo-jang/optimisation_tutorial/blob/master/tools_examples/OpEn/examples_rust/OpEn_Rust_examples_obs_avoidance_simplified.ipynb)] [[Examples Plot Visualisation](https://github.com/inmo-jang/optimisation_tutorial/blob/master/tools_examples/OpEn/examples_rust/OpEn_Rust_examples_obs_avoidance_plotcheck.ipynb)]

- [Using Rust with Jupyter Notebook](https://github.com/google/evcxr/tree/master/evcxr_jupyter) 

- You can play with rust in [Rust Online Executer](https://play.rust-lang.org/) 
