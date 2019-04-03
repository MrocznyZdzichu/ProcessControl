% b = [0.0351, 0.0007166, - 0.03438];
% a = [1, - 1.863, 0.8655];
% [z,p,k] = tf2zp(b,a);
s = tf('s');

K = [0.01613, 0.01613; 0.0351, -0.02134];
% % Kc = [2.172e-05,  2.172e-05; 4.969e-05, -3.021e-05];
q = cond(K);

LAMBDA1 = K.*(inv(K'));

sigm = svd(K);

ki1 = 0;
ki2 = 0;
kp1 = 20;
kp2 = 20;
R1=kp1*(1+(ki1/s));
R2=kp2*(1+(ki2/s));

y1r = discreteTF(1,1) * op_u(1)
y1a = (discreteTF(1,1) - (discreteTF(1,2) * discreteTF(2,1) * R2)/(1 + R2 * discreteTF(2,2)))
% G11R1=series(G11,R1); %pierwszy sk³adnik sumy % %
% G22R2=series(G22,R2); %drugi sk³adnik sumy % % G11R1ss=ss(G11R1); % %
% G22R2ss=ss(G22R2); % % W1=parallel(G11R1ss,G22R2ss); %pierwsze sumowanie
 % W1ss=ss(W1); % % GGRR1=series(G11R1ss,G22R2ss); %trzeci sk³adnik sumy
 % W2=parallel(GGRR1,W1ss); %drugie sumowanie % %
% G12G21=series(G12,G21); % % R1R2=series(R1,R2); % %
% GGRR2=series(G12G21,R1R2); % % GGRR2m=series(GGRR2,-1); %czwarty sk³adnik
% %sumy (suma z plusem) % % GGRR2mss=ss(GGRR2m); % %
% W=parallel(W2,GGRR2mss); %trzecie sumowanie (ostatnie) % % % ch-ka
% %czêstotliwoœciowa: % % wektw=logspace(-4,0,100); % 100 punktów od 10^-4
% do 10^0 % % Wfr=freqresp(W,wektw); %wektor ch-ki czêstotliwoœciowej W % %
% Wabs=abs(Wfr); % % W1=parallel(W,1); %1+W % % Wfr1=freqresp(W1,wektw);
% %wektor ch-ki czêstotliwoœciowej 1+W % % W1abs=abs(Wfr1); % %
% Lclogabs=20*(log10(Wabs)-log10(W1abs)); %wektor wartoœci funkcji
% %wskaŸnika % % Lcmax=max(Lclogabs); %wartoœæ kryterium BLT
