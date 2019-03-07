# ProcessControl
Predictive and multiloop PID control of example industrial process. Project contains linearization, discretization of process then setting the PID and MPC regulation.

## Setting environment
First you should use initialisation script. It sets all constants and operating point values.

## Simulating plant as non-linear:
Use script NonlinearSimulation.m You can set parameters by changing assigned variables within the script. To use disturbations without noise (equal to operation point values) set amplitudes variables to 0.