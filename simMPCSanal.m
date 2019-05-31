%simulation parameters
tspan = 800;

y1StepTime = 200;
y1Step = 10;

y2StepTime = 20;
y2Step = 25;

SP = zeros(tspan, 2);
SP(y1StepTime:end, 1) = SP(1, 1) + y1Step;
SP(y2StepTime:end, 2) = SP(1, 2) + y2Step;

%set disturbance signals
noiseAmpl = 0.05;
u3 = cumsum(noiseAmpl*randn(tspan, 1));
u3(u3 < -op_Fd) = -op_Fd;
u4 = cumsum(noiseAmpl*randn(tspan, 1));
u4(u4 < -op_Td) = -op_Td;

%prepare SP turbo vector
predSP = zeros(2 * horizPred, 1);
xHistory = [];
yHistory = [];

%other parameters
u1_max = 50;
u1_min = 0;
u2_max = 50;
u2_min = 0;
y1_max = 100;
y1_min = 0;
y2_max = 70;
y2_min = 25;
quality = 0;

%init plant
vk = [0; 0];
dk = [0; 0];
x = zeros(2, 1);
x_prev = zeros(2, 1);
u = [op_Fh, op_Fc, op_Fd + u3(1), op_Td + u4(1)];
delta_u = [0;0];
delta_u_noise = [delta_u', 0, 0];
anty_windup = [0, 0, 0, 0];
%iterate through all timestampls
for i = 1 : tspan-horizPred
    %compute SP turbo vector
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
    
    %wp³yw zak³óceñ stanu
    if size(u, 1) > 7
        x_pred = discreteSS.A*x_prev + discreteSS.B(:, 1:2)*...
            ([u(end, 1);u(end-7, 2)] - [op_Fh; op_Fc]);
    else
        x_pred = discreteSS.A*x_prev + discreteSS.B(:, 1:2)*...
            ([u(end, 1);u(1, 2)] - [op_Fh; op_Fc]);
    end
    vk = x - x_pred;
    
    %wp³yw opóŸnienia stan-wyjœcie
    if size(xHistory, 1) > 3
        y = xHistory(end-3, 2);
        dk = [0; y - x(2, 1)];
    end
    %compute trajektoria swobodna
    %sk³adowa od bie¿¹cego stanu
    s1 = a*x;
    %sk³adowa od dotychczasowego sterowania
    if size(u, 1) > 7
        s2 = V*discreteSS.B(:, 1:2)*...
            ([u(end-0, 1); u(end-7, 2)]...
            - [op_Fh; op_Fc]);
    else
        s2 = V*discreteSS.B(:, 1:2)*...
            ([u(end, 1); u(1, 2)] - [op_Fh; op_Fc]);
    end
    %wp³yw zak³óceñ stanu i nieliniowoœci wyjœcia
    s3 = V*(1*vk + 1*dk);
    Y0 = s1 + s2 + s3;
    delta_u = K*(predSP - Y0);
    
    if i > 1
        delta_u_noise = [delta_u', u3(i)-u3(i-1), u4(i)-u4(i-1)];
    else
        delta_u_noise = [delta_u', 0, 0];
    end
    
    u_last = u(end, :) + delta_u_noise + anty_windup;
    u_preAW = u_last;
    %limit control signals' values
    if u_last(1, 1) > u1_max
        u_last(1, 1) = u1_max;
    end
    
    if u_last(1, 1) < u1_min
        u_last(1, 1) = u1_min;
    end
    if u_last(1, 2) > u2_max
        u_last(1, 2) = u2_max;
    end
    if u_last(1, 2) < u2_min
        u_last(1, 2) = u2_min;
    end
    u = [u; u_last];
    anty_windup = u_last - u_preAW;
    
    x_prev = x;
    
    %predict next state to limit output basing on linear model
    if i > 7
        ut = [u(end, 1); u(end-7, 2); u(end, 3); u(end, 4)];
    else
        ut = [u(end, 1); u(1, 2); u(end, 3); u(end, 4)];
    end
    x_pred2 = discreteSS.A*x + discreteSS.B * (ut - op_u) + vk;
    if x_pred2(1) > y1_max - op_h
        u(end, 1:2) = discreteSS.B(:, 1:2)^(-1)* ...
            ([y1_max - op_h; x_pred2(2)] - discreteSS.A*x - vk);
        u(end, 1:2) = u(end, 1:2) + [op_Fh, op_Fc];
        
        if u(end, 1) > u1_max
            u(end, 1) = u1_max;
        end
        
        if u(end, 2) > u2_max
            u(end, 2) = u2_max;
        end
        
        if u(end, 1) < u1_min
            u(end, 1) = u1_min;
        end
        
        if u(end, 2) < u2_min
            u(end, 2) = u2_min;
        end
    end
    
    if x_pred2(1) < y1_min - op_h
        u(end, 1:2) = discreteSS.B(:, 1:2)^(-1)* ...
            ([y1_min - op_h; x_pred2(2)] - discreteSS.A*x - vk);
        u(end, 1:2) = u(end, 1:2) + [op_Fh, op_Fc];
        
        if u(end, 1) > u1_max
            u(end, 1) = u1_max;
        end
        
        if u(end, 2) > u2_max
            u(end, 2) = u2_max;
        end
        
        if u(end, 1) < u1_min
            u(end, 1) = u1_min;
        end
        
        if u(end, 2) < u2_min
            u(end, 2) = u2_min;
        end
    end
    
    if x_pred2(2) > y2_max - op_T
        u(end, 1:2) = discreteSS.B(:, 1:2)^(-1)* ...
            ([x_pred2(1); y2_max - op_T] - discreteSS.A*x - vk);
        u(end, 1:2) = u(end, 1:2) + [op_Fh, op_Fc];
        
        if u(end, 1) > u1_max
            u(end, 1) = u1_max;
        end
        
        if u(end, 2) > u2_max
            u(end, 2) = u2_max;
        end
        
        if u(end, 1) < u1_min
            u(end, 1) = u1_min;
        end
        
        if u(end, 2) < u2_min
            u(end, 2) = u2_min;
        end
    end
    
    if x_pred2(2) < y2_min - op_T
        u(end, 1:2) = discreteSS.B(:, 1:2)^(-1)* ...
            ([x_pred2(1); y2_min - op_T] - discreteSS.A*x - vk);
        u(end, 1:2) = u(end, 1:2) + [op_Fh, op_Fc];
        
        if u(end, 1) > u1_max
            u(end, 1) = u1_max;
        end
        
        if u(end, 2) > u2_max
            u(end, 2) = u2_max;
        end
        
        if u(end, 1) < u1_min
            u(end, 1) = u1_min;
        end
        
        if u(end, 2) < u2_min
            u(end, 2) = u2_min;
        end
    end
    
    %simulate plant output
    [X] = nonlinearSim3(u, x + op_X, samplingTime*i, samplingTime*(i+1), op_tauc/samplingTime, op_tau/samplingTime);
    
    x = [X(end, 1) - op_h; X(end, 2) - op_T];
    xHistory = [xHistory; x'];
    
    quality = quality + (x(1)-SP(i, 1))^2 + (x(2)-SP(i, 2))^2;
    fprintf('Iteracja nr %i\n', i)
end
quality = sqrt(quality)/tspan
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