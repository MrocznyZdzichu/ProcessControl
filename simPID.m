%setup simulation time and declutching mode
tspan = 0.5E3;
odsprzeganie = 'temperatura';      %true for using both D
                            %poziom only for D11
                            %temperatura for only D22
                            %false for none declutching
                            %% 

%configure SP series
y1SP = 0*ones(tspan, 1);
y1StepTime = 20;
y1Step = 0;

y2SP = 0*ones(tspan, 1);
y2StepTime = 20;
y2Step = 27;
%% 

%setup noises inputs 
noiseAmpl = 0.0;
u3 = cumsum(noiseAmpl*randn(tspan, 1));
u3(u3 < -op_Fd) = -op_Fd;
u4 = cumsum(noiseAmpl*randn(tspan, 1));
u4(u4 < -op_Td) = -op_Td;
%% 
%initialize variables for simulation
y = [0, 0; 0 0];               %output1, output2
e = [y1SP(1), y2SP(1);
    y1SP(2), y2SP(2)];                      %uchyb1, uchyb2
CV = [0 0; 0 0];                            %pid1, pid2

D11 = [0 0; 0 0];
D22 = [0 0; 0 0];

quality = 0;
u = [op_Fh op_Fc op_Fd op_Td; op_Fh op_Fc op_Fd op_Td];
u1max = 50;
u2max = 50;
%% 
%iterate through each sample
for i = 2:tspan
    
    %make SP step change if the time is right :)
    if i == y1StepTime
        y1SP(i:end) = y1SP(i) + y1Step;
    end
    if i == y2StepTime
        y2SP(i:end) = y2SP(i) + y2Step;
    end
    %% 
    %compute CV as controllers output
    pid1 = lsim(R12, e(:, 1));
    pid2 = lsim(R21, e(:, 2));   
    CV = [CV; [pid1(end), pid2(end)]];
    %% 
    %compute ultimate input depending on declutching type
    if strcmp(odsprzeganie, 'true')
        D22 = lsim(D22TF, CV(:, 1));
        D11 = lsim(D11TF, CV(:, 2));
        u = [u; [D22(end)+CV(end, 2)+op_Fh, D11(end)+CV(end, 1)+op_Fc], u3(i) + op_Fd, u4(i) + op_Td];
        
    elseif strcmp(odsprzeganie, 'temperatura')
        D22 = lsim(D22TF, CV(:, 1));
        u = [u; [D22(end)+CV(end, 2)+op_Fh, CV(end, 1)+op_Fc, u3(i) + op_Fd, u4(i) + op_Td]];
    elseif strcmp(odsprzeganie, 'poziom')
        D11 = lsim(D11TF, CV(:, 2));
        u = [u; [CV(end, 2)+op_Fh, D11(end)+CV(end, 1)+op_Fc, u3(i) + op_Fd, u4(i) + op_Td]];
    else
        u = [u; [CV(end, 2)+op_Fh, CV(end, 1)+op_Fc, u3(i) + op_Fd, u4(i) + op_Td]];
    end
    %% 
    %limit control signal - no negative water flow
    if u(end, 1) < 0
        u(end, 1) = 0;
    end
    if u(end, 2) < 0
        u(end, 2) = 0;
    end
    if u(end, 1) > u1max
        u(end, 1) = u1max;
    end
    if u(end, 2) > u2max;
        u(end, 2) = u2max;
    end
    %%     
    [y1, y2] = nonlinearSim2(u, op_X,(i+1), op_tauc/samplingTime, op_tau/samplingTime);
    %compute output and error of system
    y = [y; [y1(end)-op_h, y2(end)-op_T]];
    e = [e; [[y1SP(i), y2SP(i)] - y(end, :)]];
    quality = quality + (y1(end)-op_h - y1SP(i))^2 + (y2(end)-op_T - y2SP(i))^2;
    fprintf('Iteracja nr %i\n', i)
    %disp(i)
end
%% 
%compute quality of control as a mse
quality = sqrt(quality)/tspan

%% 
%plot simulation results
figure()
subplot(2, 2, 1)
plot(1:tspan, y(1:tspan, 1)+op_h, 'b', 1:tspan, y1SP + op_h, 'r')
xlabel('Numer probki sygnalu')
ylabel('Poziom wody w zbiorniku [cm]')
legend('Symulowany poziom wody', 'Zadany poziom wody')

subplot(2, 2, 2)
plot(1:tspan, y(1:tspan, 2)+op_T, 'b', 1:tspan, y2SP+op_T, 'r')
xlabel('Numer probki sygnalu')
ylabel('Temperatura wody w zbiorniku [$^{\circ}C$]')
legend('Symulowana temperatura', 'Temperatura zadana')

subplot(2, 2, 3)
plot(1:tspan, u(1:tspan, 1), 'r', 1:tspan, u(1:tspan, 2), 'b')
xlabel('Numer probki sygnalu')
ylabel('Wartosc sygnalu sterujacego [$\frac{cm}{s}$]')
legend('Doplyw wody cieplej', 'Doplyw wody zimnej')

subplot(2, 2, 4)
plot(1:tspan, u(1:tspan, 3), 'b', 1:tspan, u(1:tspan, 4), 'r')
xlabel('Numer probki sygnalu')
ylabel('Wartosc sygnalu zaklocen')
legend('Zaklocenie doplywu $[\frac{cm}{s}]$', 'Zaklocenie temperatury $[^{\circ}C]$')