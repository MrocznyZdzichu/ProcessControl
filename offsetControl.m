function deltau = offsetControl(u, op_Fh, op_Fc, op_Fd, op_Td)
%change u-vector represenation to deltau = u - op_u
deltau = u;
deltau(:, 1) = deltau(:, 1) - op_Fh;
deltau(:, 2) = deltau(:, 2) - op_Fc;
deltau(:, 3) = deltau(:, 3) - op_Fd;
deltau(:, 4) = deltau(:, 4) - op_Td;
end

