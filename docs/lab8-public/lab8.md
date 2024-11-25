---
title: Lab 8
usemathjax: true
---

<script type="text/javascript" id="MathJax-script" async
  src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js">
</script>

# Linearization of non-linear dynamics around fixed points

Find fixed points of the following systems and linearize their dynamics around these fixed points, i.e.,

- given a system \\( \dot{x} = f(x) \\), find \\(\overline{x}\\) such that \\(f(\overline{x})=0\\) and 
- formulate approximate dynamics \\(\dot{\Delta x} = A ⋅ \Delta x\\), where \\(A\\) is the matrix you need to find and \\(\Delta x=x-\overline{x}\\).

[Notes with solutions and hints](linearization-solutions.pdf).

## System 1

Consider the following 1-dimensional system that can be used to model population growth \\(x\\) is the population size, \\(P_{max}\\) is population limit above which the environment becomes resource scarse.

$$\dot{x} = f(x) = x(P_{max}-x)$$

## System 2

Damped pendulum (\\(\delta\\) is the damping coefficient), where \\(\theta\\) denotes the angle.

$$\ddot{\theta} = -\sin(\theta) - \delta\dot{\theta}$$

Denote

$$x = \begin{pmatrix} x_1 \\ x_2\end{pmatrix} = \begin{pmatrix} \theta\\ \dot{\theta} 
\end{pmatrix} $$


$$\dot{x} =  \begin{pmatrix}\dot{x_1} \\ \dot{x_2}\end{pmatrix} = \begin{pmatrix}x_2 \\ -\sin(x_1) - \delta x_2 \end{pmatrix}$$ 

which is non-linear due to the \\(\sin\\) function.

## System 3

In the following system you can assume \\(-\pi \le \theta \le \pi\\)


$$\dot{r} = r^2 - r$$

$$\dot{\theta} = \sin^2(\theta / 2)$$


## System 4

$$\begin{pmatrix}\dot{x} \\ \dot{y}\end{pmatrix} = 
\begin{pmatrix}x(3-x-2y)\\ y(2-x-y)\end{pmatrix}$$
