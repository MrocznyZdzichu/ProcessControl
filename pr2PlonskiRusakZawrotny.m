initialisation;
%simulateInOperatingPoint;          %non-linear simulation near operating point
u = CVstepChange(-15, 10, 50000, 20000, 100000, op_Fh, op_Fc);
nonlinearSim1(u, op_Fc, op_Fh, op_X, 100000, op_tauc);