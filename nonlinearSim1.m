function [h_out, T_out] = nonlinearSim1(u, op_X, stop_time, op_tauc, op_tau)

t_ = linspace(0, stop_time, stop_time);

%compute Fc as Fc = Fcin(t-tauc)
delayed_Fc = u(2, 1:op_tauc);
delayed_Fc = [delayed_Fc, u(2, 1:end-op_tauc)];

[t, X] = ode23(@(t, X)derivative_X(t, X, t_, delayed_Fc, u(1, :), u(3, :), u(4, :)), t_, op_X);

%output Tout = T(t - tau)
T_out = X(1, 2)*ones(op_tau, 1);
T_out = [T_out; X(1:end-op_tau, 2)];
h_out = X(:, 1);

ts_Fc = interp1(t_, u(2, :), t);
ts_Fh = interp1(t_, u(1, :), t);

figure(1)
subplot(3, 1, 1);
plot(t, X(:, 1))
ylabel("Wysokoœæ poziomu wody [cm]");
xlabel("Czas [s]")

subplot(3, 1, 2);
plot(t, T_out)
ylabel("Temperatura wody [$\\^\circ C$]", 'Interpreter', 'latex')
xlabel("Czas [s]")

subplot(3, 1, 3);
plot(t, ts_Fc, 'r', 'DisplayName', 'Dop³yw wody zimnej');
hold on
plot(t, ts_Fh, 'b', 'DisplayName', 'Dop³yw wody ciep³ej')
xlabel('Czas [s]');
ylabel('Wartosc doplywu sterujcego $[\frac{cm^3}{s}]$', 'Interpreter', 'latex')
legend;


end
