# TinyMPC: Model-Predictive Control on Resource-Constrained Microcontrollers 
Paper [link](https://arxiv.org/pdf/2310.16985)

## Introduction
- Want to take advantage of MPC on less powerful devices
- Assumes QP problem
- MPC has no matrix inversions and uses low memory

## Background
### LQR
- LQR looks to minimize the total cost by having some `Q` that represents cost of being at the wrong state, and some matrix `R` that represents cost of taking a large action
- Computes ideal control trajectory for entire time horizon to reach end goal (constrains solution for last value to be $x_{goal}$)
### Convex MPC
- Convex MPC is a MPC formulation that allows for control input limits. MPC also goes over some finite horizon length, without necessarily calculating the trajectory to the goal
- Can be solved as a standard QP problem if constraints are same as LQR
### ADMM (Alternating Direction Method of Multipliers)
- Optimization technique for convex optimization problems
- Given some $\min_x f(x)$, introduce some slack variable $z$ such that
$$I(z) = 0 \text{ if } z = 0, \infty \text{ else}$$
We can then reformulate the minimization problem as
$$\min_{x, x=z} f(x) + I(z)$$
By using Lagrange multipliers, we get that
$$L(x, z, \lambda) = f(x) + I(z) + \lambda^T(x-z) + \frac{\rho}{2}||x-z||^2$$
Where $\rho$ is some scaling factor to punish differences in $x$ and $z$

In traditional optimization, we would optimize for $x, z, \lambda$ all at the same time. With ADMM method, only update one variable at a time.
$$x = \argmin_x L(x, z, \lambda)$$
$$z = \argmin_z L(x, z, \lambda)$$
$$\lambda = \lambda + \rho(x + z)$$
For QP Problems, ADMM is very simple to compute (all linear models / projections)

## Method
### Combining LQR & ADMM for MPC
Take the simple LQR cost function formulation and integrate ADMM into it (solving for the Lagrangian). However, we have both $x$ and $\mu$, so it works slightly differently. We start w/ the initial Jacobian cost that MPC usually, has, then integrate other parts of ADMM into that

$$\min J(x, \mu) + I_x(z) + I_u(z) + \sum_{k=1}^{N} \frac{\rho}{2}(x_k-z_k)^T(x_k-z_k) + \lambda^T(x_k-z_k) + \sum_{k=1}^{N} \frac{\rho}{2}(u_k-w_k)^T(u_k-w_k) + \mu^T(u_k-w_k)$$

With this formulation, the function can be optimized efficiently using ADMM

### Pre-Computation
With a long enough horizon, the Riccati equation will converge to the solution of the infinite-horizon LQR Problem. Since we have an LQR base formulation, part of the update state includes the Riccati equaion. As a result, this part can be computed just once, and repeatedly used in the re-computation to save time and memory.

### Scaling factor
The computation fluctuates with the value of $\rho$ a good amount. Many solvers use a dynamic $\rho$, but thats computationally expensive. Instead, they store several precomputed matrices based on the value of $\rho$ and switch between them when needed.
