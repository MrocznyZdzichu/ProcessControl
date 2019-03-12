function y = offsetOutput(y, op_h, op_T)
y(:, 1) = y(:, 1) + op_h;
y(:, 2) = y(:, 2) + op_T;
end

