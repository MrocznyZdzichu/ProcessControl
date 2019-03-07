# ProcessControl
Predictive and multiloop PID control of example industrial process. Project contains linearization, discretization of process then setting the PID and MPC regulation.

## Setting environment
First you should use initialisation script. It sets all constants and operating point values.

## Simulating plant as non-linear:
Use script NonlinearSimulation.m You can set parameters by changing assigned variables within the script. To use disturbations without noise (equal to operation point values) set amplitudes variables to 0.

## Getting a linear state-space model:
Just run Linearize script (after initialisation!). It will return `LinearModel` object which is a ready state-space model.
It is described by `A_ss`, `B_ss`, `C_ss`, and `D_ss` matrices.

### Simulations:
To simulate you need to prepare control vector first. You can use `CVstepChange` function to set step changingu-vector.
To test it with state-space model, you need to offset u first, so it would relate to operating point. You can do it easily
by `offsetControl' method. Simulation could be run by lsim method - remember to set zero initial conditions (zero = operating point).
