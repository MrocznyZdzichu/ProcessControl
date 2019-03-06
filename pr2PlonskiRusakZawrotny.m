initialisation;
%simulateInOperatingPoint;          %non-linear simulation near operating point

step_Fh = -15;
step_Fc = 10;
stepTime_Fh = 5e4;
stepTime_Fc = 2e4;
timeRange = 1e5;
Fd_amplitude = 4;

u = CVstepChange(step_Fh, step_Fc, stepTime_Fh, stepTime_Fc, timeRange, op_Fh, op_Fc);
nonlinearSim1(u, op_X, 100000, op_tauc, op_tau);