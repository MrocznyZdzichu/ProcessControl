%simulation parameters
tspan = 0.5E3;

y1StepTime = 50;
y1Step = 25;

y2StepTime = 150;
y2Step = 0;

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
delta_u_max = 0.5;
u1_max = 70;
u1_min = 0;
u2_max = 70;
u2_min = 0;
%iterate through all timestampls
for i = 1 : tspan-horizPred
    %on 1st step initialise sim
    if i == 1
        v = [0; 0];
        x = zeros(2, 1);
        u = [op_Fh, op_Fc, op_Fd + u3(i), op_Td + u4(i)];
        delta_u = [0;0];
        delta_u_noise = [delta_u', 0, 0];
        anty_windup = [0, 0, 0, 0];
    else
        %compute SP turbo vector
        if i == 60
            debugAssign = 1
        end
        if i == y1StepTime
            for j = 1 : horizPred
                predSP(2*j-1, 1)    = y1Step;
            end  
        end
        if i == y2StepTime
            for j = 1 : horizPred
                predSP(2*j, 1)      = y2Step;
            end
        end
        
        %compute control increment
        s1 = c*a*x;
        s2 = c*V*discreteSS.B(:, 1:2)*(u(i, 1:2)' - [op_Fh; op_Fc]);
        s3 = c*V*v;
        delta_u = K*(predSP - s1 - s2 - s3);
        
        %delta_u values limit
        for k = 1:2
            if delta_u(k) > 1
                delta_u(k) = delta_u_max;
            end
            if delta_u(k) < -1
                delta_u(k) = -1 * delta_u_max;
            end
        end
        delta_u_noise = [delta_u', u3(i)-u3(i-1), u4(i)-u4(i-1)];
        
    end
    u_last = u(end, :) + delta_u_noise + anty_windup;
    u_preAW = u_last;
    %limit control signals' values
    if u_last(1, 1) > u1_max
        u_last(1, 1) = u1_max;
    elseif u_last(1, 1) < u1_min
        u_last(1, 1) = u1_min;
    elseif u_last(1, 2) > u2_max
        u_last(1, 2) = u2_max;
    elseif u_last(1, 2) < u2_min
        u_last(1, 2) = u2_min;
    end
    u = [u; u_last];
    anty_windup = u_last - u_preAW;
    
    %predict state as x(k+1) = Ax(k) + Bu(k)
    xp = discreteSS.A * x + discreteSS.B(:, 1:2) * (u_last(1, 1:2)' - op_u(1:2, 1));
    
    %simulate plant output and update state and prediction error
    [X] = nonlinearSim3(u, x + op_X, samplingTime*i, samplingTime*(i+1), op_tauc/samplingTime, op_tau/samplingTime);
    
    x = [X(end, 1) - op_h; X(end, 2) - op_T];
    xHistory = [xHistory; x'];
    v = -1*(x - xp);
    
    fprintf('Iteracja nr %i\n', i)
end
%compute output basing on state
yHistory = zeros(size(xHistory));
yHistory(:, 1) = xHistory(:, 1) + op_h;
yHistory(1:op_tau/samplingTime, 2) = op_T * ones(op_tau/samplingTime, 1);
yHistory(op_tau/samplingTime + 1:end, 2) = xHistory(1:end-(op_tau/samplingTime), 2) + op_T;

%plot simulation results
figure()
subplot(2, 2, 1)
plot(1:tspan-horizPred, yHistory(:, 1), 'b', 1:tspan-horizPred, SP(1:tspan-horizPred, 1) + op_h, 'r')
xlabel('Numer probki sygnalu')
ylabel('Poziom wody w zbiorniku [cm]')
legend('Symulowany poziom wody', 'Zadany poziom wody')

subplot(2, 2, 2)
plot(1:tspan-horizPred, yHistory(:, 2), 'b', 1:tspan-horizPred, SP(1:tspan-horizPred, 2) + op_T, 'r')
xlabel('Numer probki sygnalu')
ylabel('Temperatura wody w zbiorniku [$^{\circ}C$]')
legend('Symulowana temperatura', 'Temperatura zadana')

subplot(2, 2, 3)
plot(1:tspan-horizPred, u(1:tspan-horizPred, 1), 'r', 1:tspan-horizPred, u(1:tspan-horizPred, 2), 'b')
xlabel('Numer probki sygnalu')
ylabel('Wartosc sygnalu sterujacego [$\frac{cm}{s}$]')
legend('Doplyw wody cieplej', 'Doplyw wody zimnej')

subplot(2, 2, 4)
plot(1:tspan-horizPred, u(1:tspan-horizPred, 3), 'b', 1:tspan-horizPred, u(1:tspan-horizPred, 4), 'r')
xlabel('Numer probki sygnalu')
ylabel('Wartosc sygnalu zaklocen')
legend('Zaklocenie doplywu $[\frac{cm}{s}]$', 'Zaklocenie temperatury $[^{\circ}C]$')