R21 = pidtune(discreteTF(2, 1), 'pidf');
R21.Kp = 1;
R21.Ki = 8e-5;
R12 = pidtune(discreteTF(1, 2), 'pidf');
R12.Kp = 1;
R12.Ki = 10e-6;