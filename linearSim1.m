function [] = linearSim1(Fc_amplitude, Fh_amplitude, op_Fc, op_Fh, op_X, stop_time)
    t_Fc = linspace(0, stop_time, stop_time*6);             %time vectors for time-dependent control
    t_Fh = linspace(0, stop_time, stop_time*6);
    
    ts_Fc = zeros(stop_time*6, 1);
    ts_Fh = zeros(stop_time*6, 1);
    ts_Fc(1:floor(end/2)) = op_Fc;                                 %step change of control variable by given amplitude
    ts_Fc(floor(end/2)+1:end) = op_Fc + Fc_amplitude;
    
    ts_Fh(1:floor(end/2)) = op_Fh;
    ts_Fh(floor(end/2+1):end) = op_Fh + Fh_amplitude;
    
    tspan = [0, stop_time];
    [t, X] = ode45(@(t, X)derivative_X(t, X, t_Fc, ts_Fc, t_Fh, ts_Fh), tspan, op_X);
    
    figure()
    plot(t, X(:, 1))
    hold on
    plot(t, ts_Fc)
    hold on
    plot(t, ts_Fh)
end
