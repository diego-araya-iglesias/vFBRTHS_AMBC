clc, clear, close all, format shortG
syms s me ce ke a1b0 a2 a3 b1 b2 mp kc g

A = 1/(s+a3);
SV = a1b0/(s^2+b1*s+b2);
CSI = a2*s;
P = 1/(mp*s^2);
%%  kc = 0 / E = ke
E = 1;
Gini = E*A*SV/(1+E*A*SV+E*A*CSI);
disp(' ');disp('============================================== // ==============================================');disp(' ');
disp('Gini =');disp(' ');
pretty(collect(Gini));

%%  kc = 0 / E  = 1/[me ce ke]
E = 1/(me*s^2+ce*s+ke);
Ge = E*A*SV/(1+E*A*SV+E*A*CSI);
disp(' ');disp('============================================== // ==============================================');disp(' ');
disp('Ge =');disp(' ');
pretty(collect(Ge));

%%  kc ~= 0 / E  = 1/[me ce ke]
E = 1/(me*s^2+ce*s+ke);
Gp = P*A*SV/(1+P*A*SV+P*A*CSI+P*kc/(1+kc*E));
disp(' ');disp('============================================== // ==============================================');disp(' ');
disp('Gp =');disp(' ');
pretty(collect(Gp));
