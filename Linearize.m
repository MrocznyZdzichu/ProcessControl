%prepare variables for symbolic math
syms h;
syms T;
syms Fh;
syms Fc;
syms Fd;
syms Th;
syms Tc
syms Td;

%prepare anonymous versions of state and output functions

f1 = 1/(3*6*h^2)*(Fh + Fd + Fc) - 9/(3*6*h^1.5);                        %dh/dt
f2 = 1/(6*h^3)*(Fh*Th + Fc*Tc + Fd*Td - T*(Fh + Fc + Fd));              %dT/dt

%compute Taylor series as a symbolic expressions using f(x0) ~~ 0

a11 = diff(f1, h);
a12 = diff(f1, T);
a21 = diff(f2, h);
a22 = diff(f2, T);
A = [a11 a12; a21 a22];

b11 = diff(f1, Fh);
b12 = diff(f1, Fc);
b13 = diff(f1, Fd);
b14 = diff(f1, Td);
b21 = diff(f2, Fh);
b22 = diff(f2, Fc);
b23 = diff(f2, Fd);
b24 = diff(f2, Td);
B = [b11 b12 b13 b14; b21 b22 b23 b24];
% set symbolic variables to their values in operating point

h = op_h;
T = op_T;
Fh = op_Fh;
Fc = op_Fc;
Fd = op_Fd;
Th = op_Th;
Tc = op_Tc;
Td = op_Td;

%compute final space state matrices

A_ss = double(subs(A))
B_ss = double(subs(B))
C_ss = eye(2)
D_ss = zeros(2, 4)

LinearModel = ss(A_ss, B_ss, C_ss, D_ss, 'InputDelay', [0; op_tauc; 0; 0], 'OutputDelay', [0; op_tau]);