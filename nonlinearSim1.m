function [h_out, T_out] = nonlinearSim1(u, op_X, stop_time, op_tauc, op_tau)

t_ = 100*linspace(0, stop_time, stop_time)';

%compute Fc as Fc = Fcin(t-tauc)
delayed_Fc = zeros(numel(u(:, 2)), 1);
for i = 1:size(u, 1)
    if i > op_tauc
        delayed_Fc(i) = u(i-op_tauc, 2);
    else
        delayed_Fc(i) = u(1, 2);
    end
end
[t, X] = ode23(@(t, X)derivative_X(t, X, t_, delayed_Fc, u(:, 1), u(:, 3), u(:, 4)), t_, op_X);
h_out = X(:, 1);
T_out = X(:, 2);
%output Tout = T(t - tau)
for i = 1:size(u, 1)
    if i > op_tau
        T_out(i) = X(i-op_tau, 2);
    else
        T_out(i) = X(1, 2);
    end
end
end

