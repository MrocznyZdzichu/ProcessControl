%setup control parameters
horizControl    = 10;
horizPred       = 12;
psi1            = 3.5;
psi2            = 1;
lambda1         = 100;
lambda2         = 120;
psi = [];
lambda = [];
%build weight diagonal matrices
for i = 1 : 2*horizPred
    if mod(i, 2) == 1
        psi(i, i) = psi1;
    else
        psi(i, i) = psi2;
    end
end
for i = 1 : 2*horizControl
    if mod(i, 2) == 1
        lambda(i, i) = lambda1;
    else
        lambda(i, i) = lambda2;
    end
end

%compute dynamic matrix M
M = dynamicMatrix(horizPred, horizControl,...
    discreteSS.A, discreteSS.B(1:2, 1:2), discreteSS.C);

%for better numeric arithmetics inv(k) by qr factorization
k = M' * psi * M + lambda;
[Q, R] = qr(k);
K = (R^-1 * Q') * M' * psi;

%take 2 (count of control inputs) first rows of
K = K([1, 2], :);

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

