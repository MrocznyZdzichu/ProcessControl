clear;
C = 6;
alpha = 9;
C_ss = eye(2);          %state vector is output of system
D_ss = zeros(2);        %output is independent of control

% op_prefix stands for operating point

op_Tc = 19;             %input, cold water temperature in degC
op_Th = 81;             %input, hot water temperature in degC
op_Td = 42;             %disturbance, water temperature in degC
op_Fc = 31e-0;          %control, cold water flow in cm3/s
op_Fh = 19e-0;          %control, hot water flow in cm3/s
op_Fd = 14e-0;          %disturbance, water flow in cm3/s
op_tauc = 21e3;         %latency of input cold water flow in s
op_tau = 6e3;           %latency of output water flow in s
op_h = 50.57;          %output, water level in cm
op_T = 42.44;           %output, temperature of output water

op_u = [op_Fh; op_Fc];
op_X = [op_h; op_T];