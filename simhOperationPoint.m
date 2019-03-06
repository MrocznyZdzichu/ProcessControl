tspan = [0, 10];
h0 = op_h;
[t, h] = ode45(@(t, h)derivative_h_still(t, h), tspan, h0);
figure(1);
plot(t,h)