function dh = derivative_h_still(t, h)

%method describing derivative of water level in tank
%in operating point it is equal to 0.0014 ~~ 0
%used to simulate in operating point

dh = 1/(3*6*h^2)*(19+14+31) - 9/(3*6*h^3/2);
end

