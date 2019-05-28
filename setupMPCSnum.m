%setup control parameters
horizControl    = 10;
horizPred       = 12;
weightError     = 0.2;
weightControl   = 0.45;

%build weight diagonal matrices
psi     = weightError*eye(2*horizPred);
lambda  = weightControl*eye(2*horizControl);

%compute dynamic matrix M
M = dynamicMatrix(horizPred, horizControl,...
    discreteSS.A, discreteSS.B(1:2, 1:2), discreteSS.C);

%precalculate H and A matrices
H = 2*(M'*psi*M+lambda);

J = zeros(2*horizControl, 2*horizControl);
for i = 1 : horizControl
    J(2*i-1:2*i, 1:2*i) = 1;
end
A = [-1*J; J; -M; M];