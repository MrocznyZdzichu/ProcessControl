function [] = nonlinearSim1(u, op_Fc, op_Fh, op_X, stop_time, op_tauc)
    t_Fc = linspace(0, stop_time, stop_time);                    %time vectors for time-dependent control
    t_Fh = linspace(0, stop_time, stop_time);
      
    tspan = linspace(0, stop_time, stop_time);
    delayed_Fc = u(2, 1:op_tauc);
    delayed_Fc = [delayed_Fc, u(2, 1:end-op_tauc)];
    [t, X] = ode23(@(t, X)derivative_X(t, X, t_Fc, delayed_Fc, t_Fh, u(1, :)), tspan, op_X);
    
    ts_Fc = interp1(t_Fc, u(2, :), t);
    ts_Fh = interp1(t_Fh, u(1, :), t);
    
    figure(1)
    subplot(2, 1, 1);
    plot(t, X(:, 1))
    subplot(2, 1, 2);
    plot(t, ts_Fc)
    hold on
    plot(t, ts_Fh)
    
    figure(2)
    subplot(2, 1, 1);
    plot(t, X(:, 2))
    subplot(2, 1, 2);
    plot(t, ts_Fc)
    hold on
    plot(t, ts_Fh)
end
