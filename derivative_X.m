function dX = derivative_X(t, X, t_, ts_Fc, ts_Fh, ts_Fd, ts_Td)
%time series for time-changing variables
ts_Fc = interp1(t_, ts_Fc, t);
ts_Fh = interp1(t_, ts_Fh, t);
ts_Fd = interp1(t_, ts_Fd, t);
ts_Td = interp1(t_, ts_Td, t);

dX = zeros(2, 1);
dX(1) = 1/(3*6*X(1)^2)*(ts_Fh + ts_Fd + ts_Fc) - 9/(3*6*X(1)^(3/2));
dX(2) = 1/(6*X(1)^3)*(ts_Fh*81 + ts_Fc*19 + ts_Fd*ts_Td - X(2)*(ts_Fh + ts_Fc + ts_Fd));
end

