tspan = [0, 10];
X0 = [op_h; op_T];
[t, X] = ode45(@(t, X)derivative_X_still(t, X), tspan, X0);

figure(1)
plot(t, X(:, 1))
title("Przebieg wysokoœci w poziomu wody w otoczeniu punktu pracy")
xlabel("czas [s]")
ylabel("wysokoœæ poziomu wodu [cm]")
axis([0 10 30 70])

figure(2)
plot(t, X(:, 2))
title("Przebieg temperatury wyjœciowej w otoczeniu punktu pracy")
xlabel("czas [s]")
ylabel("temperatura wody [DEGC]")
axis([0 10 30 50])