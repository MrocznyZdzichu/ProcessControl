samplingTime = 3000;
Ad = (eye(size(A_ss, 1)) + 0.5*A_ss*samplingTime)*inv((eye(size(A_ss, 1)) - 0.5*A_ss*samplingTime));
Bd = inv(A_ss)*(Ad - eye(size(Ad, 1)))*B_ss;
discreteSS = ss(Ad, Bd, C_ss, D_ss, samplingTime, 'InputDelay', [0; op_tauc; 0; 0]/samplingTime, 'OutputDelay', [0; op_tau]/samplingTime)
discreteTF = tf(discreteSS)