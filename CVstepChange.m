function u = CVstepChange(stepFh, stepFc, stepTimeFh, stepTimeFc, stopTime, op_Fh, op_Fc, step_Fd, step_Td, stepTime_Fd, stepTime_Td)
u = zeros(2, stopTime);                    %setting CVs for simulation
u(1, :) = [linspace(op_Fh, op_Fh, stepTimeFh), linspace(op_Fh+stepFh, op_Fh + stepFh, stopTime - stepTimeFh)];
u(2, :) = [linspace(op_Fc, op_Fc, stepTimeFc), linspace(op_Fc+stepFc, op_Fc + stepFc, stopTime - stepTimeFc)];
u(3, :) = [linspace(14, 14, stepTime_Fd), linspace(14 + step_Fd, 14 + step_Fd, stopTime - stepTime_Fd)];
u(4, :) = [linspace(42, 42, stepTime_Td), linspace(42 + step_Td, 42 + step_Td, stopTime - stepTime_Td)];
end

