function dX = derivative_X_MPC(t, X, ts_Fc, ts_Fh, ts_Fd, ts_Td)

dX = zeros(2, 1);
dX(1) = 1/(3*6*X(1)^2)*(ts_Fh + ts_Fd + ts_Fc) - 9/(3*6*X(1)^(3/2));
dX(2) = 1/(6*X(1)^3)*(ts_Fh*81 + ts_Fc*19 + ts_Fd*ts_Td - X(2)*(ts_Fh + ts_Fc + ts_Fd));
end

