tspan = [0, 10];
T0 = op_T;
[t, T] = ode45(@(t, T)derivative_T_still(t, T), tspan, T0);
figure(2);
plot(t,T)