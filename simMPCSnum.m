%simulation parameters
tspan = 0.5E3;

y1StepTime = 20;
y1Step = 0;

y2StepTime = 20;
y2Step = 0;

SP = zeros(tspan, 2);
SP(y1StepTime:end, 1) = SP(1, 1) + y1Step;
SP(y2StepTime:end, 2) = SP(1, 2) + y2Step;

%set disturbance signals
% noiseAmpl = 0.0;
% u3 = cumsum(noiseAmpl*randn(tspan, 1));
% u3(u3 < -op_Fd) = -op_Fd;
% u4 = cumsum(noiseAmpl*randn(tspan, 1));
% u4(u4 < -op_Td) = -op_Td;

%prepare SP turbo vector
predSP = zeros(2 * horizPred, 1);
xHistory = [];
yHistory = [];
quality = 0;
%other parameters
delta_u_max = 2;
u1_max = 50;
u1_min = 0;
u2_max = 50;
u2_min = 0;
y1_max = 75;
y1_min = 0;
y2_max = 70;
y2_min = 25;

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
        dk = [0; 0];
        vk = [0; 0];
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
        %         for k = 1:2
        %             if delta_u(k, 1) > 1
        %                 delta_u(k, 1) = delta_u_max;
        %             end
        %             if delta_u(k, 1) < -1
        %                 delta_u(k, 1) = -1 * delta_u_max;
        %             end
        %         end
        u_last = u(end, :) + [delta_u(1:2, 1)', ...
            u3(i) - u3(i-1), u4(i) - u4(i-1)];
        u = [u; u_last];
        
        x_prev = x;
        %simulate plant output and update state and prediction error
        [X] = nonlinearSim3(u, x + op_X, samplingTime*i, samplingTime*(i+1), op_tauc/samplingTime, op_tau/samplingTime);
        
        x = [X(end, 1) - op_h; X(end, 2) - op_T];
        xHistory = [xHistory; x'];
        
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