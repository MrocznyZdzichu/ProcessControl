function dt = derivative_T(C, h, Fh, Th, Fc, Tc, Fd, Td, T)
%method describing derivative of output water temperature
%in operating point equals -2e-7 ~~ 0 :)
dt = 1/(C*h^3)*(Fh*Th + Fc*Tc + Fd*Td - T*(Fh + Fc + Fd));
end

