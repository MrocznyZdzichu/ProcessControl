%setup control parameters
horizControl    = 10;
horizPred       = 12;
weightError     = 1;
weightControl   = 0.05;

%build weight diagonal matrices
psi     = weightError*eye(2*horizPred);
lambda  = weightControl*eye(2*horizControl);

%compute dynamic matrix M
M = dynamicMatrix(horizPred, horizControl,...
    discreteSS.A, discreteSS.B(1:2, 1:2), discreteSS.C);

%for better numeric arithmetics inv(k) by qr factorization
k = M' * psi * M + lambda;
[Q, R] = qr(k);
K = (R^-1 * Q') * M' * psi;

%take 2 (count of control inputs) first rows of
K = K(1:2, :);

% concatenate MPCS-version of plant matrices
c = eye(2 * horizPred);

aElement = discreteSS.A;
a = [];

VElement = zeros(size(discreteSS.A));
V = [];
for i = 1 : horizPred
    a = [a; aElement^i];
    VElement = VElement + discreteSS.A^(i-1);
    V = [V; VElement];
end

