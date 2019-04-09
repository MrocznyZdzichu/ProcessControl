# ProcessControl
Predictive and multiloop PID control of example industrial process. Project contains linearization, discretization of process then setting the PID and MPC regulation.

## Setting environment
First you should use initialisation script. It sets all constants and operating point values.

## Simulating plant as non-linear:
Use script NonlinearSimulation.m You can set parameters by changing assigned variables within the script. To use disturbations without noise (equal to operation point values) set amplitudes variables to 0.

## Getting a linear models:
Just run Linearize script (after initialisation!). It will return `LinearModel` object which is a ready state-space model.
It is described by `A_ss`, `B_ss`, `C_ss`, and `D_ss` matrices. Additionaly transfer function G_LinearModel will be returned.

## Getting a discrete models;
Call `discretization` script. Within the script you can adjust sampling time by changing `samplingTime` assginment.
Discrete models in space-state `discreteSS` and as transfer function `discreteTF` will be returned. Using this script will perform
MATLAB's `c2d` function and will return `C` and `D` matrices changed. To avoid this run `discretizationManual` script. It will
compute `A` and `B` matrices and WILL NOT change `C` and `D`.
 
### Simulations:
To simulate you need to prepare control vector first. You can use `CVstepChange` function to set step changingu-vector.
To test it with state-space model, you need to offset u first, so it would relate to operating point. You can do it easily
by `offsetControl` method. Simulation could be run by lsim method - remember to set zero initial conditions (zero = operating point).
G_LinearModel is a transfer function model. It can be simulated with `lsim` too or by computing product with input in Laplace domain.
Example test is made in `compareModels' script.