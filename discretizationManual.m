samplingTime = 1500;
Ad = (eye(size(A_ss, 1)) + 0.5*A_ss*samplingTime)/(eye(size(A_ss, 1)) - 0.5*A_ss*samplingTime);
Bd = inv(A_ss)/(Ad - eye(size(Ad, 1)))*B_ss;
discreteSS = ss(Ad, Bd, C_ss, D_ss, samplingTime)
disreteTF = tf(discreteSS)