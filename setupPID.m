R21 = pidtune(discreteTF(2, 1), 'pidf');
R21.Kp = 2.25;
R21.Ki = 9e-5;
R12 = pidtune(discreteTF(1, 2), 'pidf');
R12.Kp = 0.75;
R12.Ki = 9e-6;
