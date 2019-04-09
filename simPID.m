y1SP = -50;
y2SP = 100;
y1_init = 0;
y2_init = 0;
tspan = 2E3;


y = [y1_init, y2_init];                 %output1, output2
e = [y1SP - y1_init, y2SP - y2_init;
     y1SP - y1_init, y2SP - y2_init];   %uchyb1, uchyb2
CV = [0, 0;
      0, 0];                            %u1, u2
for i = 2:tspan
    pid1 = lsim(R12, e(:, 1));
    pid2 = lsim(R21, e(:, 2));
    if (i == 1)
        CV = [pid2(end), pid1(end)];
    else
        CV = [CV; [pid2(end), pid1(end)]];
    end
    
    G12 = lsim(discreteTF(1, 2), CV(:, 2), [], y2_init);
    G22 = lsim(discreteTF(2, 2), CV(:, 2));
    G21 = lsim(discreteTF(2, 1), CV(:, 1), [], y1_init);
    G11 = lsim(discreteTF(1, 1), CV(:, 2));
    
    %y = [y; [G12(end), G21(end)]];
    y = [y; [G12(end)+G11(end), G21(end)+G22(end)]];
    e = [e; [[y1SP, y2SP] - y(end, :)]];
    disp(i)
end