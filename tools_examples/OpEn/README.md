# OpEn 

## Introduction 

**[OpEn](https://alphaville.github.io/optimization-engine/)** is a "fast and accurate embedded optimization engine for robotics and autonomous systems". Its detailed was published in [1]. 

This framework is based on PANOC (**P**roximal **A**veraged **N**ewton-type method for **O**ptimal **C**ontrol) [2]

Its application examples for collision avoidances are described in [3][4]. 




## Examples: How to use 

As of Apr 2020, OpEn provides 3 types of usages: (1) Rust; (2); Python API; (3) MATLAB API. The followings show examples of how to use OpEn by each of the ways. From my experience, using rust directly provides much faster than Python API for a simple optimisation problem. 

  - [Rust](https://github.com/inmo-jang/optimisation_tutorial/blob/master/tools_examples/OpEn/examples_rust/OpEn_Rust_example.ipynb)

  - [Python API](https://github.com/inmo-jang/optimisation_tutorial/blob/master/tools_examples/OpEn/examples_python/OpEn_Python_Panelty.ipynb)

  - MATLAB API (TODO)
    
    
## References

[1] [Sopasakis et al., 2020, "OpEn: Code Generation for Embedded Nonconvex Optimization"](https://arxiv.org/abs/2003.00292)

[2] [Stella et al,, 2017, "A Simple and Efficient Algorithm for Nonlinear Model Predictive Control"](https://arxiv.org/abs/1709.06487)

[3] [Small et al., 2018, "Aerial navigation in obstructed environments with embedded nonlinear model predictive control"](https://arxiv.org/abs/1812.04755)

[4] [Sathya et al., 2019, "Embedded nonlinear model predictive control for obstacle avoidance using PANOC"](https://arxiv.org/abs/1904.10546)
