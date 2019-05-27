function [X_out] = nonlinearSim3(u, init_state, start_time, stop_time, op_tauc, op_tau)

t_ = 1*[start_time, stop_time];

%compute Fc as Fc = Fcin(t-tauc)
delayed_Fc = zeros(numel(u(:, 2)), 1);
for i = 1:size(u, 1)
    if i > op_tauc
        delayed_Fc(i) = u(i-op_tauc, 2);
    else
        delayed_Fc(i) = u(1, 2);
    end
end
[t, X] = ode45(@(t, X)derivative_X_MPC(t, X, delayed_Fc(end), u(end, 1), u(end, 3), u(end, 4)),...
    t_, init_state);
X_out = X;
end

