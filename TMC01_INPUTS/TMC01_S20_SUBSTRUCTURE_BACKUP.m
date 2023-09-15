%   TITLE     : ROBUST ADAPTIVE COMPENSATION OF FBRTHS TESTING WITH UNCERTAIN COMPLIANCE SPRING AND FORCE MEASUREMENT NOISE
%   SUBJECT   : INPUTS - REFERENCE STRUCTURE
%   AUTHOR    : DIEGO ARAYA IGLESIAS
%% INITIALIZING
clc;clear;close all;format shortG;
path = 'C:\MATLAB\THESIS_MATLAB_CODE';
addpath(genpath(path));
%% SUBSTRUCTURING
load('TMC01_MF01_REFERENCE_STRUCTURE.mat');
% EXPERIMENTAL SUBSTRUCTURE
me = mr(1);
ke = kr(1);
ce = Cr(1,1)-Cr(2,2)+Cr(3,3);
% EXPERIMENTAL STATE-SPACE
ssAe = [0,1;-ke/me,-ce/me];
ssBe = [0;1/me];
ssCe = [1,0;0,1;-ke/me,-ce/me];
ssDe = [0;0;1/me];
sse  = ss(ssAe,ssBe,ssCe,ssDe);
% NUMERICAL SUBSTRUCTURE
Mn    = Mr(2:end,2:end);
Kn    = Kr(2:end,2:end);
Cn    = Cr(2:end,2:end);
ndofn = size(Mn,1);
iotan = ones(ndofn,1);
% NUMERICAL STATE-SPACE
Mne  = [Mr(1,1) 0; 0 0];
Kne  = [-Kr(2,1) 0;-Kr(3,1) 0];
Cne  = [-Cr(2,1) 0;-Cr(3,1) 0];
ssAn = [zeros(ndofn),eye(ndofn);-Mn\Kn,-Mn\Cn];
Bn1  = [zeros(ndofn,1);-(Mn\Mn)*iotan];
Bn2  = [zeros(ndofn,1);Mn\Kne*iotan];
Bn3  = [zeros(ndofn,1);Mn\Cne*iotan];
ssBn = [Bn1,Bn2,Bn3];
Cn1  = [eye(ndofn),zeros(ndofn)];
Cn2  = [Kne,Cne];
Cn3  = [-Kn,-Cn];
ssCn = [Cn1;Cn2;Cn3];
Dn1  = zeros(2,3);
Dn2  = [-Mne*iotan,-Kne*iotan,-Cne*iotan];
Dn3  = [zeros(ndofn,1),Kne*iotan,Cne*iotan];
ssDn = [Dn1;Dn2;Dn3];
ssN  = ss(ssAn,ssBn,ssCn,ssDn);
% Mne  = [Mr(1,1) 0; 0 0];
% Kne  = [-Kr(2,1);-Kr(3,1)];
% Cne  = [-Cr(2,1);-Cr(3,1)];
% ssAn = [zeros(ndofn),eye(ndofn);-Mn\Kn,-Mn\Cn];
% Bn1  = [zeros(ndofn,1);-(Mn\Mn)*iotan];
% Bn2  = [zeros(ndofn,1);Mn\Kne];
% Bn3  = [zeros(ndofn,1);Mn\Cne];
% ssBn = [Bn1,Bn2,Bn3];
% Cn1  = [eye(ndofn),zeros(ndofn)];
% Cn2  = [diag(Kne),diag(Cne)];
% Cn3  = [-Kn,-Cn];
% ssCn = [Cn1;Cn2;Cn3];
% Dn1  = zeros(2,3);
% Dn2  = [-Mne*iotan,-Kne,-Cne];
% Dn3  = [zeros(ndofn,1),Kne,Cne];
% ssDn = [Dn1;Dn2;Dn3];
% ssN  = ss(ssAn,ssBn,ssCn,ssDn);

% TRANSFER SYSTEM PARAMETERS (SERVO-HYDRAULIC ACTUATOR)
a1b0 = 2.13e13;
a2   = 4.23e6;
a3   = 3.3;
b1   = 425;
b2   = 1e5;
mp   = 1;
kp   = 0;
cp   = 0;
ssAp = [0,1;-kp/mp,-cp/mp];
ssBp = [0;1/mp];
ssCp = [1,0;0,1];
ssDp = [0;0];
% SAVE RESULTS
clear ndofr iotar Mr Kr Cr zr ssAr ssBr ssCr ssDr kw
save('TMC01_MF02_SUBSTRUCTURE.mat');
%% END
rmpath(genpath(path));