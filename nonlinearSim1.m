function [] = nonlinearSim1(u, op_X, stop_time, op_tauc, op_tau, Fd_amplitude, op_Fd, Td_amplitude, op_Td)
%
t_ = linspace(0, stop_time, stop_time);
tspan = linspace(0, stop_time, stop_time);

%compute Fc as Fc = Fcin(t-tauc)
delayed_Fc = u(2, 1:op_tauc);
delayed_Fc = [delayed_Fc, u(2, 1:end-op_tauc)];

%compute disturbances basing on given amplitudes
Fd = op_Fd + Fd_amplitude*2*rand(1, size(t_, 2)) - Fd_amplitude;
Td = op_Td + Td_amplitude*2*rand(1, size(t_, 2)) - Td_amplitude;

[t, X] = ode23(@(t, X)derivative_X(t, X, t_, delayed_Fc, u(1, :), Fd, Td), tspan, op_X);

%output Tout = T(t - tau)
T_out = X(1, 2)*ones(op_tau, 1);
T_out = [T_out; X(1:end-op_tau, 2)];

ts_Fc = interp1(t_, u(2, :), t);
ts_Fh = interp1(t_, u(1, :), t);

figure(1)
subplot(2, 1, 1);
plot(t, X(:, 1))
ylabel("WysokoSC poziomu wody [cm]");
xlabel("Czas [s]")

subplot(2, 1, 2);
plot(t, ts_Fc, 'r', 'DisplayName', 'Dop造w wody zimnej');
hold on
plot(t, ts_Fh, 'b', 'DisplayName', 'Dop造w wody ciep貫j')
xlabel('czas [s]');
ylabel('Wartosc doplywu sterujcego $[\frac{cm^3}{s}]$', 'Interpreter', 'latex')
legend;

figure(2)
subplot(2, 1, 1);
plot(t, T_out)
ylabel('Temperatura wody na wyjsciu $[^\circ C]$', 'Interpreter', 'latex');
xlabel('Czas [s]')

subplot(2, 1, 2);
plot(t, ts_Fc, 'r', 'DisplayName', 'Dop造w wody zimnej');
hold on
plot(t, ts_Fh, 'b', 'DisplayName', 'Dop造w wody ciep貫j')
xlabel('czas [s]');
ylabel('Wartosc doplywu sterujacego $[\frac{cm^3}{s}]$', 'Interpreter', 'latex')
legend;
end
