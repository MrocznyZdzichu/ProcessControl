R21 = pidtune(discreteTF(2, 1), 'pidf');
R21.Kp = 2.25;
R12 = pidtune(discreteTF(1, 2), 'pidf');
R12.Kp = 0.75;
