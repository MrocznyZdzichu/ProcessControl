function dh = derivative_h(h, C, Fh, Fd, Fc, alpha)
%method describing derivative of water level in tank
%in operating point it is equal to 0.0014 ~~ 0
dh = 1/(3*C*h^2)*(Fh+Fd+Fc) - alpha/(3*C*h^3/2);
end

