timeFlag = 'continuous';            %continuous or discrete
modelType = 'ss';                   %ss or tf

step_Fh = -15;            %step value for hot water supply
step_Fc = 20;           %step value for cold water supply
stepTime_Fh = 2e4;      %step time for hot water supply
stepTime_Fc = 2e4;    %step time for cold water supply
timeRange = 1e5;        %time span of simulation
Fd_amplitude = 5;       %amplitude of water flow disturbance
Td_amplitude = 5;       %amplitude of temperature disturbance

%prepare input for nonlinear and linear model
u = CVstepChange(step_Fh, step_Fc, stepTime_Fh, stepTime_Fc, timeRange, op_Fh, op_Fc);
u = addOscilations(u, Fd_amplitude, Td_amplitude, timeRange);
linear_u = offsetControl(u', op_Fh, op_Fc, op_Fd, op_Td);

%timespan for ode solving
t_ = linspace(0, timeRange, timeRange);

%compute Fc as Fc = Fcin(t-tauc)
delayed_Fc = u(2, 1:op_tauc);
delayed_Fc = [delayed_Fc, u(2, 1:end-op_tauc)];

%compute output of nonlinear plant
[t, X] = ode23(@(t, X)derivative_X(t, X, t_, delayed_Fc, u(1, :), u(3, :), u(4, :)), t_, op_X);
figure(1)
plot(t, X(:, 1))

%compute output of linear plant
if timeFlag == 'continuous'
   if modelType == 'ss'
       [y, linear_t, x] = lsim(LinearModel, linear_u, t_);
       y = offsetOutput(y, op_h, op_T);
       hold on
       plot(t, y(:, 1))
   elseif modelType == 'tf'
       
   end
elseif timeFlag == 'discrete'
    
end