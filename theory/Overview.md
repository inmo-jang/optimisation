# Optimisation Methods: A Brief Overview


## (1) Unconstrained

### Descent Algorithms: General
- Given  $$\min{x} f(x),$$


- Descent algorithms generally **pick directions $p_k$ (and step-size $a_k$) iteratively** such that $f(x_k + \alpha_k p_k) < f(x_k)$, for some $\alpha_k > 0$. 



- **Direction $p_k$ selection**: Depending on how to find $p_k$, there are many variants such as Gradient Descent, Conjugate Gradient, Newton, etc..

- **Step-size $\alpha_k$ selection**: This becomes the minimisation problem called **Line Search**: 
$$\min_{ \alpha_{k} > 0} f(x_k + a_k p_k)$$

    - But, if we want to find $a_k$ exactly at once, then this problem is expensive. So, we need clever ways (i.e. solve it approximately and cheaply -- called **Inexact line search**).

    - **Backgracking**
    - (Not completed yet)

- The update direction and step size are the primary things that motivated multiple variants of descent algorithms. 

- Materials to read
    - [CMU lecture](http://www.cs.cmu.edu/~pradeepr/convexopt/Lecture_Slides/Descent-Line-Search.pdf)
    


### [Gradient Descent](http://www.cs.cmu.edu/~pradeepr/convexopt/Lecture_Slides/Gradient-Descent.pdf)

- **Direction**: $p = -\frac{\nabla f_k}{\| \nabla f_k \|}$ (unit direction $p$ with most rapid decrease; first-order method that uses gradient)

- "Gradient method is often **slow**; convergence varies depending on scale". As this method relies on **local information**.
    
- "Gradient method does not handle **non-differentiable problems**"

- **Other methods with improved convergence**

    - quasi-Newton methods
    - conjugate gradient method
    - accelerated gradient method

- **Other methods for nondierentiable or constrained problems**

    - subgradient method
    - proximal gradient method
    - smoothing methods
    - cutting-plane methods

![Image description](images/Gradient_Newton.png)    