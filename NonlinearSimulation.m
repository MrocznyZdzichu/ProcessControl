%simulateInOperatingPoint;          %non-linear simulation near operating point

step_Fh = 5;
step_Fc = -7;
stepTime_Fh = 2e4;
stepTime_Fc = 2.5e4;
timeRange = 1e5;
Fd_amplitude = 0;
Td_amplitude = 0;

u = CVstepChange(step_Fh, step_Fc, stepTime_Fh, stepTime_Fc, timeRange, op_Fh, op_Fc);
nonlinearSim1(u, op_X, 100000, op_tauc, op_tau, Fd_amplitude, op_Fd, Td_amplitude, op_Td);