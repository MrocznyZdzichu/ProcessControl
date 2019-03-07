function u = CVstepChange(stepFh, stepFc, stepTimeFh, stepTimeFc, stopTime, op_Fh, op_Fc)
u = zeros(2, stopTime);                    %setting CVs for simulation
u(1, :) = [linspace(op_Fh, op_Fh, stepTimeFh), linspace(op_Fh+stepFh, op_Fh + stepFh, stopTime - stepTimeFh)];
u(2, :) = [linspace(op_Fc, op_Fc, stepTimeFc), linspace(op_Fc+stepFc, op_Fc + stepFc, stopTime - stepTimeFc)];
u(3, :) = 14;
u(4, :) = 42;
end

