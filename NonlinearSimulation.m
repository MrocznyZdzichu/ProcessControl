%simulateInOperatingPoint;          %non-linear simulation near operating point

step_Fh = -15;            %step value for hot water supply
step_Fc = 20;           %step value for cold water supply
stepTime_Fh = 2e4;      %step time for hot water supply
stepTime_Fc = 2e4;    %step time for cold water supply
timeRange = 1e5;        %time span of simulation
Fd_amplitude = 5;       %amplitude of water flow disturbance
Td_amplitude = 5;       %amplitude of temperature disturbance

u = CVstepChange(step_Fh, step_Fc, stepTime_Fh, stepTime_Fc, timeRange, op_Fh, op_Fc);
u = addOscilations(u, Fd_amplitude, Td_amplitude, timeRange);

nonlinearSim1(u, op_X, timeRange, op_tauc, op_tau);