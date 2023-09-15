%   TITLE     : ROBUST ADAPTIVE COMPENSATION OF FBRTHS TESTING WITH UNCERTAIN COMPLIANCE SPRING AND FORCE MEASUREMENT NOISE
%   SUBJECT   : INPUTS - TRANSFER FUNCTIONS
%   AUTHOR    : DIEGO ARAYA IGLESIAS
%% INITIALIZING
clc;clear;close all;format shortG;
path = 'C:\MATLAB\THESIS_MATLAB_CODE';
addpath(genpath(path));
%% LOAD INPUTS
load('TMC01_MF02_SUBSTRUCTURE.mat');
%% TRANSFER FUNCTION: Gini(s)
Gini.A3 = a2+1;
Gini.A2 = a3+b1+a2*b1;
Gini.A1 = b2+a2*b2+a3*b1;
Gini.A0 = a1b0+a3*b2;
% GpIni(s)
Gini.B  = a1b0;
Gini.A  = [Gini.A3,Gini.A2,Gini.A1,Gini.A0];
Gini.tf = tf(Gini.B,Gini.A);
Gini.tf = minreal(Gini.tf);
disp(' ');disp('============================================== // ==============================================');disp(' ');
disp('Gini(s) = ');disp(' ');display(Gini.tf);
%% TRANSFER FUNCTION: Ge(s)
% INPUTS
Ge.A5 = me;
Ge.A4 = ce + me*a3 + me*b1;
Ge.A3 = ke + ce*a3 + a2 + b1*(ce + me*a3) + me*b2;
Ge.A2 = ke*a3 + b1*(ke + ce*a3 + a2) + b2*(ce + me*a3);
Ge.A1 = ke*a3*b1 + b2*(ke + ce*a3 + a2);
Ge.A0 = ke*a3*b2 + a1b0;
% Gpe(s)
Ge.B  = a1b0;
Ge.A  = [Ge.A5,Ge.A4,Ge.A3,Ge.A2,Ge.A1,Ge.A0];
Ge.tf = tf(Ge.B,Ge.A);
Ge.tf = minreal(Ge.tf,2);
disp(' ');disp('============================================== // ==============================================');disp(' ');
disp('Ge(s) = ');display(Ge.tf);
%% TRANSFER FUNCTION: Gp(s)
% COMPLIANCE SPRING
kc = 5/100*ke;
% INPUTS
Gp.A7 = me*mp;
Gp.A6 = (a3*me+b1*me+ce)*mp;
Gp.A5 = (a2*me+a3*(b1*me+ce)*mp+b1*ce*mp+b2*me*mp+kc*(me+mp)+ke*mp);
Gp.A4 = (a2*(b1*me+ce)+a3*(b1*ce*mp+b2*me*mp+kc*(me+mp)+ke*mp)+b1*(kc*(me+mp)+ke*mp)+(b2*mp+kc)*ce);
Gp.A3 = (a2*(b1*ce+b2*me+kc+ke)+a3*(b1*(kc*(me+mp)+ke*mp)+(b2*mp+kc)*ce)+b1*ce*kc+b2*(kc*(me+mp)+ke*mp)+kc*ke);
Gp.A2 = (a1b0*me+a2*(b1*(kc+ke)+b2*ce)+a3*(b1*ce*kc+b2*(kc*(me+mp)+ke*mp)+kc*ke)+(b1*ke+b2*ce)*kc);
Gp.A1 = (a1b0*ce+a2*b2*(kc+ke)+(a3*(b1*ke+b2*ce)+b2*ke)*kc);
Gp.A0 = a1b0*(kc+ke)+a3*b2*kc*ke;
% Gp(s)
Gp.B   = a1b0*[me,ce,ke+kc];
Gp.A   = [Gp.A7,Gp.A6,Gp.A5,Gp.A4,Gp.A3,Gp.A2,Gp.A1,Gp.A0];
Gp.tf2 = tf(Gp.B,Gp.A);
Gp.tf  = minreal(Gp.tf2,2);
Gp.tf2 = minreal(Gp.tf2);
disp(' ');disp('============================================== // ==============================================');disp(' ');
disp('Gp(s) = ');display(Gp.tf2);
disp('Gp(s) = ');display(Gp.tf);
%% TRANSFER FUNCTION: Gest(s)
% ESTIMATED WITH 3 POLES AND 0 ZEROS
[Gest.data,Gest.B,Gest.A] = estimate_transfer_function(5,8192,0.1,20,3,0,Gp.tf);
Gest.tf                   = tf(Gest.B,Gest.A);
Gest.tf                   = minreal(Gest.tf);
disp(' ');disp('============================================== // ==============================================');disp(' ');
disp('Gest(s) = ');display(Gest.tf);
%% IFF
sampling_frequency   = 4096;
sampling_period      = 1/sampling_frequency;
[Derivatives,~,~] = finite_difference_matrix(3,1,sampling_period,-1);
coeff             = flip(Gest.A/Gest.B);
Kff               = coeff*Derivatives;
KFFz  = tf(Kff,1,sampling_period,'Variable','z^-1');
KFFs  = d2c(KFFz,'tustin');
Lp    = minreal(KFFs*Gest.tf);
% INITIAL COEFFICIENTS FOR AMBC
AambIni = flip(Gini.A/Gini.B); % Initial compensator's parameters ai
display(AambIni);
% NOISE FILTER FOR ADAPTIVE PROCESS
BWorder               = 4; % order
BWfcut                = 20; % cutoff frequency
[NumFilter,DenFilter] = butter(BWorder,BWfcut/(4096/2)); % discrete filter coefficients
BWtf                  = tf(NumFilter,DenFilter);

%% SAVING RESULTS
save('TMC01_MF03_TRANSFER_FUNCTIONS.mat','Gini','Ge','Gp','Gest','Kff','Derivatives','NumFilter','DenFilter','AambIni');
%% END
rmpath(genpath(path));