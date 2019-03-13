samplingTime = 10;
discreteSS = c2d(LinearModel, samplingTime, 'tustin')
discreteTF = c2d(G_LinearModel, samplingTime, 'tustin')