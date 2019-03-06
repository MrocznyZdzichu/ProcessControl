function dX = derivative_X(t, X, t_Fc, ts_Fc, t_Fh, ts_Fh)

ts_Fc = interp1(t_Fc, ts_Fc, t);
ts_Fh = interp1(t_Fh, ts_Fh, t);
dX = zeros(2, 1);
dX(1) = 1/(3*6*X(1)^2)*(ts_Fh+14+ts_Fc) - 9/(3*6*X(1)^(3/2));
dX(2) = 1/(6*X(1)^3)*(ts_Fh*81 + ts_Fc*19 + 14*42 - X(2)*(ts_Fh + ts_Fc + 14));
end

