%simulation parameters
tspan = 0.3E3;

y1StepTime = 50;
y1Step = 0;

y2StepTime = 50;
y2Step = -15;

SP = zeros(tspan, 2);
SP(y1StepTime:end, 1) = SP(1, 1) + y1Step;
SP(y2StepTime:end, 2) = SP(1, 2) + y2Step;

%set disturbance signals
noiseAmpl = 0.0;
u3 = cumsum(noiseAmpl*randn(tspan, 1));
u4 = cumsum(noiseAmpl*randn(tspan, 1));

%prepare SP turbo vector
predSP = zeros(2 * horizPred, 1);
xHistory = [];
yHistory = [];

%other parameters
delta_u_max = 0.2;
u1_max = 70;
u1_min = 0;
u2_max = 70;
u2_min = 0;
y1_max = 70;
y1_min = 0;
y2_max = 100;
y2_min = 27;

%create constraints turbo-vectors
Umin = [];
Umax = [];
Ymin = [];
Ymax = [];

for i = 1 : horizControl
    Umin = [Umin; [u1_min-op_Fh; u2_min - op_Fc]];
    Umax = [Umax; [u1_max-op_Fh; u2_max - op_Fc]];
end
for i = 1 : horizPred
    Ymin = [Ymin; [y1_min-op_h; y2_min - op_T]];
    Ymax = [Ymax; [y1_max-op_h; y2_max - op_T]];
end
for i = 1 : tspan - horizPred
    %initialise state in 1st sample
    if i == 1
        v = [0; 0];
        x = zeros(2, 1);
        u = [op_Fh, op_Fc, op_Fd + u3(i), op_Td + u4(i)];
    else
        %get SP turbo-vector
        if i == y1StepTime+1
            for j = 1 : horizPred
                predSP(2*j-1, 1)    = y1Step;
            end
        end
        if i == y2StepTime+1
            for j = 1 : horizPred
                predSP(2*j, 1)      = y2Step;
            end
        end
        %get trajektoria swobodna
        Y0 = [];
        factor = zeros(2, 2);
        for j = 1 : horizPred
            factor = factor + discreteSS.A^(j-1);
            Y0 = [Y0; discreteSS.C*discreteSS.A^j*x + ...
                discreteSS.C*factor*...
                (discreteSS.B(:, 1:2)*(u(end, 1:2)'-[op_Fh; op_Fc])+v)];
        end
        
        f = -2*M'*psi*(predSP - Y0);
        
        %get constraing matrix b
        U =[];
        for j = 1 : horizControl
            U = [U; u(end, 1:2)'- [op_Fh; op_Fc]];
        end
        b = [-1*Umin + U;
            Umax - U;
            -1*Ymin+Y0;
            Ymax - Y0];
        
        delta_u = quadprog(H, f, A, b);
        %evaluate delata_u values
        for k = 1:2
            if delta_u(k, 1) > 1
                delta_u(k, 1) = delta_u_max;
            end
            if delta_u(k, 1) < -1
                delta_u(k, 1) = -1 * delta_u_max;
            end
        end
        u_last = u(end, :) + [delta_u(1:2, 1)', ...
            u3(i) - u3(i-1), u4(i) - u4(i-1)];
        u = [u; u_last];
        
        %predict state as x(k+1) = Ax(k) + Bu(k)
        xp = discreteSS.A * x + discreteSS.B(:, 1:2) * (u_last(1, 1:2)' - op_u(1:2, 1));
        
        %simulate plant output and update state and prediction error
        [X] = nonlinearSim3(u, x + op_X, samplingTime*i, samplingTime*(i+1), op_tauc/samplingTime, op_tau/samplingTime);
        
        x = [X(end, 1) - op_h; X(end, 2) - op_T];
        xHistory = [xHistory; x'];
        v = -1*(x - xp);
        quality = quality + (x(1)-SP(i, 1))^2 + (x(2)-SP(i, 2))^2;
    end  
    fprintf('Iteracja nr %i\n', i)
end
quality = sqrt(quality)/tspan
%compute output basing on state
yHistory = zeros(size(xHistory));
yHistory(:, 1) = xHistory(:, 1) + op_h;
yHistory(1:3, 2) = op_T * ones(3, 1);
yHistory(4:end, 2) = xHistory(1:end-3, 2) + op_T;

%plot simulation results
figure()
subplot(2, 2, 1)
plot(1:tspan-horizPred-1, yHistory(:, 1), 'b', 1:tspan-horizPred-1, SP(1:tspan-horizPred-1, 1) + op_h, 'r')
xlabel('Numer probki sygnalu')
ylabel('Poziom wody w zbiorniku [cm]')
legend('Symulowany poziom wody', 'Zadany poziom wody')

subplot(2, 2, 2)
plot(1:tspan-horizPred-1, yHistory(:, 2), 'b', 1:tspan-horizPred-1, SP(1:tspan-horizPred-1, 2) + op_T, 'r')
xlabel('Numer probki sygnalu')
ylabel('Temperatura wody w zbiorniku [$^{\circ}C$]')
legend('Symulowana temperatura', 'Temperatura zadana')

subplot(2, 2, 3)
plot(1:tspan-horizPred-1, u(1:tspan-horizPred-1, 1), 'r', 1:tspan-horizPred-1, u(1:tspan-horizPred-1, 2), 'b')
xlabel('Numer probki sygnalu')
ylabel('Wartosc sygnalu sterujacego [$\frac{cm}{s}$]')
legend('Doplyw wody cieplej', 'Doplyw wody zimnej')

subplot(2, 2, 4)
plot(1:tspan-horizPred-1, u(1:tspan-horizPred-1, 3), 'b', 1:tspan-horizPred-1, u(1:tspan-horizPred-1, 4), 'r')
xlabel('Numer probki sygnalu')
ylabel('Wartosc sygnalu zaklocen')
legend('Zaklocenie doplywu $[\frac{cm}{s}]$', 'Zaklocenie temperatury $[^{\circ}C]$')