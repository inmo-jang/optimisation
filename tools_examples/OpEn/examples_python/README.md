# OpEn Python Examples

- All the subfolders are examples using Python Rust interface for various optimisation problems. 

- Briefly speaking, you need to make a python file for **code generation (for solver)**, and another for **calling the solver**. For details, you need to read [this](https://alphaville.github.io/optimization-engine/docs/python-interface).

- For OpEn Python Interface, you need to have **Python version 3.5 or newer** (See [this](https://alphaville.github.io/optimization-engine/docs/installation#python-interface) for details)

## How to use it

- Step 1) Code Generation by ``python codegen_solver.py`` (Note that it takes some time). Then, a new directory ``python_build`` was built. 

- Step 2) Run the solver by ``python call_solver.py``. Then, you might get the following result:

    ```
    {'Pong': 1}
    Solution: x =  [-0.8143262975451454, -0.46488157789626977, -0.6217488270466801, -0.4479993439349785, -1.0]
    Solver_time: =  0.196455
    ```


## Examples

[01)](https://github.com/inmo-jang/optimisation_tutorial/tree/master/tools_examples/OpEn/examples_python/example_rosenbrock) Rosenbrock function

[02)](https://github.com/inmo-jang/optimisation_tutorial/tree/master/tools_examples/OpEn/examples_python/example_02) $\min (x-p_1)^2$, subject to $x < p_2$

