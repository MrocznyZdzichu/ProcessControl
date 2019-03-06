function dt = derivative_T_still(t, T)
%method describing derivative of output water temperature
%in operating point equals -2e-7 ~~ 0 :)
%used to simulate in operating point
dt = 1/(6*50.57^3)*(19*81 + 31*19 + 14*42 - T*(19 + 31 + 14));
end

