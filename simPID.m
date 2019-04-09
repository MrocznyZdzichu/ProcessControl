tspan = 3E3;
odsprzeganie = 'true';

y1SP = 0*ones(tspan, 1);
y1StepTime = 200;
y1Step = 20;

y2SP = 0*ones(tspan, 1);
y2StepTime = 500;
y2Step = 15;

noiseAmpl = 0.1;
u3 = cumsum(noiseAmpl*randn(tspan, 1));
G13 = lsim(discreteTF(1, 3), u3);
G23 = lsim(discreteTF(2, 3), u3);

u4 = cumsum(noiseAmpl*randn(tspan, 1));
G14 = lsim(discreteTF(1, 4), u4);
G24 = lsim(discreteTF(2, 4), u4);

y = [y1_init, y2_init];                 %output1, output2
e = [y1SP(1), y2SP(1);
    y1SP(2), y2SP(2)];                       %uchyb1, uchyb2
CV = [0, 0;
    0, 0];                            %pid1, pid2

D11 = [0, 0; 0, 0];
D22 = [0, 0; 0, 0];

u = [0, 0;                              %u1, u2
    0, 0];
for i = 2:tspan
    if i == y1StepTime
        y1SP(i:end) = y1SP(i) + y1Step;
    end
    if i == y2StepTime
        y2SP(i:end) = y2SP(i) + y2Step;
    end
    
    pid1 = lsim(R12, e(:, 1));
    pid2 = lsim(R21, e(:, 2));
    
    CV = [CV; [pid1(end), pid2(end)]];
    
    if strcmp(odsprzeganie, 'true')
        D22 = lsim(D22TF, CV(:, 1));
        D11 = lsim(D11TF, CV(:, 2));
        u = [u; [D22(end)+CV(end, 2), D11(end)+CV(end, 1)]];
        
    elseif strcmp(odsprzeganie, 'temperatura')
        D22 = lsim(D22TF, CV(:, 1));
        u = [u; [D22(end)+CV(end, 2), CV(end, 1)]];
    elseif strcmp(odsprzeganie, 'poziom')
        D11 = lsim(D11TF, CV(:, 2));
        u = [u; [CV(end, 2), D11(end)+CV(end, 1)]];
    else
        u = [u; [CV(end, 2), CV(end, 1)]];
    end
    
    if u(end, 1) < -1*op_Fh
        u(end, 1) = -1*op_Fh;
    end
    if u(end, 2) < -1*op_Fc
        u(end, 2) = -1*op_Fc;
    end
    
    G12 = lsim(discreteTF(1, 2), u(:, 2));
    G22 = lsim(discreteTF(2, 2), u(:, 2));
    G21 = lsim(discreteTF(2, 1), u(:, 1));
    G11 = lsim(discreteTF(1, 1), u(:, 1));
    
    y = [y; [G12(end)+G11(end)+G13(i)+G14(i), G21(end)+G22(end)+G23(i)+G24(i)]];
    e = [e; [[y1SP(i), y2SP(i)] - y(end, :)]];
    %disp(i)
end
quality = [sum(e(:, 1).^2)/tspan, sum(e(:, 2).^2)/tspan];
quality = sqrt(quality(1)^2 + quality(2)^2)

figure()
subplot(2, 2, 1)
plot(1:tspan, y(:, 1)+op_h, 'b.', 1:tspan, y1SP + op_h, 'r')
xlabel('Numer probki sygnalu')
ylabel('Poziom wody w zbiorniku [cm]')
legend('Symulowany poziom wody', 'Zadany poziom wody')

subplot(2, 2, 2)
plot(1:tspan, y(:, 2)+op_T, 'b.', 1:tspan, y2SP+op_T, 'r')
xlabel('Numer probki sygnalu')
ylabel('Temperatura wody w zbiorniku [$^{\circ}C$]')
legend('Symulowana temperatura', 'Temperatura zadana')

subplot(2, 2, 3)
plot(1:tspan, u(1:tspan, 1)+op_Fh, 'r', 1:tspan, u(1:tspan, 2)+op_Fc, 'b')
xlabel('Numer probki sygnalu')
ylabel('Wartosc sygnalu sterujacego [$\frac{cm}{s}$]')
legend('Doplyw wody cieplej', 'Doplyw wody zimnej')

subplot(2, 2, 4)
plot(1:tspan, u3+op_Fd, 'b', 1:tspan, u4+op_T, 'r')
xlabel('Numer probki sygnalu')
ylabel('Wartosc sygnalu zaklocen')
legend('Zaklocenie doplywu $[\frac{cm}{s}]$', 'Zaklocenie temperatury $[^{\circ}C]$')