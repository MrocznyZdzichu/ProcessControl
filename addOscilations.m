function [u] = addOscilations(u, Fd_amplitude, Td_amplitude, timeRange)
u(3, :) = u(3, :) + Fd_amplitude*2*rand(1, timeRange) - Fd_amplitude;
u(4, :) = u(4, :) + Td_amplitude*2*rand(1, timeRange) - Td_amplitude;
end

