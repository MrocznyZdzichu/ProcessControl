G = discreteTF;
G(:, :).InputDelay = zeros(4, 1);
G(:, :).OutputDelay = zeros(2, 1);

D22TF = -1*G(2, 2)/G(2, 1);
D11TF = -1*G(1, 1)/G(1, 2);

delay22 = (discreteTF(2, 2).InputDelay+discreteTF(2, 2).OutputDelay) ...
                    - (discreteTF(2, 1).OutputDelay + discreteTF(2, 1).InputDelay);
if (delay22 > 0)
   D22TF.OutputDelay = delay22 
end
                
delay11 = (discreteTF(1, 1).InputDelay+discreteTF(1, 1).OutputDelay) ...
                    - (discreteTF(1, 2).OutputDelay + discreteTF(1, 2).InputDelay);  
if (delay11 > 0)
    D11TF.OutputDelay = delay11
end