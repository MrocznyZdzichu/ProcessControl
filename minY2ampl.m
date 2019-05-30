quality_best = 100;
psi1_max = 5;
psi2_max = 5;
lambda1_max = 150;
lambda2_max = 150;
psi1_min = 2;
psi2_min = 2;
lambda1_min = 100;
lambda2_min = 100;
for z = 1:500
    psi1        = (psi1_max-psi1_min)*rand(1) + psi1_min;
    psi2        = (psi2_max-psi2_min)*rand(1) + psi2_min;
    lambda1     = (lambda1_max-lambda1_min)*rand(1) + lambda1_min;
    lambda2     = (lambda2_max-lambda2_min)*rand(1) + lambda2_min;
    
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
    
    simMPCSanal;
    ampl = max(xHistory(:, 2)) - min(xHistory(:, 2));
    
    obj = 0.25*ampl + 2*quality;
    if obj<quality_best
        quality_best    = obj;
        psi1_best       = psi1;
        psi2_best       = psi2;
        lambda1_best    = lambda1;
        lambda2_best    = lambda2;
    end
    fprintf("Iteracja nr %i\n", z)
end