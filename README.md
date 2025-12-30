# Gantry-Systems

## Overview
This repository contains a first-principles nonlinear dynamic model and nonlinear model predictive control implementation for gantry crane systems with suspended loads.

The primary objective is point-to-point motion of the payload without residual oscillations.

- The 2D gantry system is fully modeled, controlled and simulated.
- The 3D gantry system is under active development.

The goal is for the payload to move from point to point without any residual oscillation.


## 3D Gantry System (Work in Progress)

The modeling of the 3D gantry system is still under development. Currently two modeling approaches are being considered:

* Extension of the current analytical Euler-Lagrange framework
* Modeling via rigid-body dynamics libraries (e.g. [Pinocchio](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/devel/doxygen-html/index.html))


