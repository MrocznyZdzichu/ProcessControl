R21 = pidtune(discreteTF(2, 1), 'pi');
%R21.Kp = 5;
%R21.Ki = 8e-5;
R12 = pidtune(discreteTF(1, 2), 'pi');
%R12.Kp = 3;
%R12.Ki = 7.5e-6;