timeFlag = 'discrete';            %continuous or discrete
modelType = 'tf';                   %ss or tf

step_Fh = 0;            %step value for hot water supply
step_Fc = 0;           %step value for cold water supply
stepTime_Fh = 2e4;      %step time for hot water supply
stepTime_Fc = 1e4;    %step time for cold water supply
timeRange = 1e5;        %time span of simulation
Fd_amplitude = 0;       %amplitude of water flow disturbance
Td_amplitude = 0;       %amplitude of temperature disturbance
step_Fd = -10;          %step of disturbace flow
step_Td = 5;           %step of disturbance temperature
stepTime_Fd = 2.5e4;    %step time for disturbance of flow
stepTime_Td = 1.5e4;      %step time for temperature disturbance

%prepare input for nonlinear and linear model
u = CVstepChange(step_Fh, step_Fc, stepTime_Fh, stepTime_Fc, timeRange, op_Fh, op_Fc, step_Fd, step_Td, stepTime_Fd, stepTime_Td);
u = addOscilations(u, Fd_amplitude, Td_amplitude, timeRange);
linear_u = offsetControl(u', op_Fh, op_Fc, op_Fd, op_Td);

%timespan for ode solving
t_ = linspace(0, timeRange, timeRange);

%compute Fc as Fc = Fcin(t-tauc)
delayed_Fc = op_Fc * ones(1, op_tauc);
delayed_Fc = [delayed_Fc, u(2, 1:end-op_tauc)];

%compute output of nonlinear plant
[t, X] = ode23(@(t, X)derivative_X(t, X, t_, delayed_Fc, u(1, :), u(3, :), u(4, :)), t_, op_X);
%output Tout = T(t - tau)
T_out = op_T*ones(op_tau, 1);
T_out = [T_out; X(1:end-op_tau, 2)];

figure(1)
subplot(2, 2, 1);
plot(t, X(:, 1), 'r', 'DisplayName', 'Nonlinear plant')

%compute output of linear plant
if strcmp(timeFlag, 'continuous')
    if strcmp(modelType, 'ss')
        title('Porównanie modelu nieliniowego, z ci¹g³ym modelem opisanym liniowymi równaniami stanu')
        [y, linear_t, x] = lsim(LinearModel, linear_u, t_);              
    elseif strcmp(modelType, 'tf')
        title('Porównanie modelu nieliniowego, z ci¹g³ym modelem w postaci transmitancji')
        [y, linear_t, x] = lsim(G_LinearModel, linear_u, t_);
    end
    y = offsetOutput(y, op_h, op_T); 
    hold on
    plot(linear_t, y(:, 1), 'b', 'DisplayName', 'Linearized plant')
    legend();
    xlabel('Czas [s]');
    ylabel('Wysokoœæ poziomu wody [cm]')
    
    subplot(2, 2, 2)
    plot(t_, T_out, 'r', 'DisplayName', 'Nonlinear plant')
    hold on
    plot(linear_t, y(:, 2), 'b', 'DisplayName', 'Linearized plant')
    legend();
    xlabel('Czas [s]')
    ylabel('Temperatura wody [$^\circ C$]', 'Interpreter', 'latex')
    
    subplot(2, 2, 3)
    plot(t, u(1, :), 'r', 'DisplayName', 'Dop³yw wody ciep³ej')
    hold on
    plot(t, u(2, :), 'b', 'DisplayName', 'Dop³yw wody zimnej')
    legend();
    xlabel('Czas [s]')
    ylabel('Dop³yw steruj¹cy [cm/s]')
    
    subplot(2, 2, 4)
    plot(t_, u(3, :), 'b', 'DisplayName', 'Zak³ócenie dop³ywu [cm/s]')
    hold on
    plot(t_, u(4, :), 'r', 'DisplayName', 'Zak³ócenie temperatury [DEGC]')
    legend();
    xlabel('Czas [s]')
    ylabel('Wartoœæ zak³ócenia')
elseif strcmp(timeFlag, 'discrete')
    if strcmp(modelType, 'ss')
        title('Porównanie modelu nieliniowego, z dyskretnym modelem opisanym liniowymi równaniami stanu')
        [y, linear_t, x] = lsim(discreteSS, downsample(linear_u, samplingTime));              
    elseif strcmp(modelType, 'tf')
        title('Porównanie modelu nieliniowego, z dyskretnym modelem w postaci transmitancji')
        [y, linear_t, x] = lsim(discreteTF, downsample(linear_u, samplingTime));
    end
    y = offsetOutput(y, op_h, op_T); 
    hold on
    plot(linear_t, y(:, 1), '.b', 'DisplayName', 'Linearized plant')
    legend();
    xlabel('Czas [s]');
    ylabel('Wysokoœæ poziomu wody [cm]')
    
    subplot(2, 2, 2)
    plot(t, T_out, 'r', 'DisplayName', 'Nonlinear plant')
    hold on
    plot(linear_t, y(:, 2), '.b', 'DisplayName', 'Linearized plant')
    legend();
    xlabel('Czas [s]')
    ylabel('Temperatura wody [$^\circ C$]', 'Interpreter', 'latex')
    
    subplot(2, 2, 3)
    plot(t, u(1, :), 'r', 'DisplayName', 'Dop³yw wody ciep³ej')
    hold on
    plot(t, u(2, :), 'b', 'DisplayName', 'Dop³yw wody zimnej')
    legend()
    xlabel('Czas [s]')
    ylabel('Dop³yw steruj¹cy [cm/s]')
    
    subplot(2, 2, 4)
    plot(t, u(3, :), 'b', 'DisplayName', 'Zak³ócenie dop³ywu [cm/s]')
    hold on
    plot(t, u(4, :), 'r', 'DisplayName', 'Zak³ócenie temperatury [DEGC]')
    legend()
    xlabel('Czas [s]')
    ylabel('Wartoœæ zak³ócenia')
end