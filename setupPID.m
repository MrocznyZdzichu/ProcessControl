R21 = pidtune(discreteTF(2, 1), 'pidf');
R21.Ki = 8e-5;
R21.Kd = 1e4;
R12 = pidtune(discreteTF(1, 2), 'pidf');
R12.Kp = 1.25;
R12.Kd = 1e4;