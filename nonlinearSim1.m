function [] = nonlinearSim1(u, op_X, stop_time, op_tauc, op_tau)
    t_ = linspace(0, stop_time, stop_time);                    %time vectors for time-dependent control   
    tspan = linspace(0, stop_time, stop_time);
    
    delayed_Fc = u(2, 1:op_tauc);
    delayed_Fc = [delayed_Fc, u(2, 1:end-op_tauc)];
    [t, X] = ode23(@(t, X)derivative_X(t, X, t_, delayed_Fc, u(1, :)), tspan, op_X);
    
    T_out = X(1, 2)*ones(op_tau, 1);            %T_out is T delayed by op_tau
    T_out = [T_out; X(1:end-op_tau, 2)];
    
    ts_Fc = interp1(t_, u(2, :), t);
    ts_Fh = interp1(t_, u(1, :), t);
    
    figure(1)
    subplot(2, 1, 1);
    plot(t, X(:, 1))
    subplot(2, 1, 2);
    plot(t, ts_Fc)
    hold on
    plot(t, ts_Fh)
    
    figure(2)
    subplot(2, 1, 1);
    plot(t, T_out)
    subplot(2, 1, 2);
    plot(t, ts_Fc)
    hold on
    plot(t, ts_Fh)
end
