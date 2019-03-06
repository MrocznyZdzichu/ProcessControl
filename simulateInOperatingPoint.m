tspan = [0, 10];
X0 = [op_h; op_T];
[t, X] = ode45(@(t, X)derivative_X_still(t, X), tspan, X0);

figure(1)
plot(t, X(:, 1))
title("Przebieg wysoko�ci w poziomu wody w otoczeniu punktu pracy")
xlabel("czas [s]")
ylabel("wysoko�� poziomu wodu [cm]")
axis([0 10 30 70])

figure(2)
plot(t, X(:, 2))
title("Przebieg temperatury wyj�ciowej w otoczeniu punktu pracy")
xlabel("czas [s]")
ylabel("temperatura wody [DEGC]")
axis([0 10 30 50])