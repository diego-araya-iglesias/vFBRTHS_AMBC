%   TITLE     : ROBUST ADAPTIVE COMPENSATION OF FBRTHS TESTING WITH UNCERTAIN COMPLIANCE SPRING AND FORCE MEASUREMENT NOISE
%   SUBJECT   : INPUTS - REFERENCE STRUCTURE
%   AUTHOR    : DIEGO ARAYA IGLESIAS
%% INITIALIZING
clc;clear;close all;format shortG;
path = 'C:\MATLAB\THESIS_MATLAB_CODE';
addpath(genpath(path));
%% REFERENCE STRUCTURE
% DEGREE OF FREEDOMS
mr    = [98.3 98.3 98.3];
kr    = [(0.516+20*0.516) 0.684 0.684]*1e6;
ndofr = length(mr);
iotar = diag(eye(ndofr));
% PHILLIPS: MULTI-ACTUATOR NONLINEAR NUMERICAL STUDY (EXPERIMENTAL SUBSTRUCTURE)
Mr = diag(mr)
Kr = K_matrix_ensamble(kr)

% RAYLEIGH DAMPING
% zr = [5;5]/100;
% Cr = rayleigh_damping(Mr,Kr,zr);

% MODAL DAMPING
% zr = 1/100*[0.31;0.62;0.63];
zr = 1/100*[5;5;5];
Cr = modal_damping(Mr,Kr,zr)

% STATE-SPACE 
ssAr = [zeros(ndofr),eye(ndofr);-Mr\Kr,-Mr\Cr];
ssBr = [zeros(ndofr,1);-(Mr\Mr)*iotar];
ssCr = [eye(ndofr),zeros(ndofr);-Kr,-Cr];
ssDr = [zeros(ndofr,1);zeros(ndofr,1)];
ssr  = ss(ssAr,ssBr,ssCr,ssDr);
disp(' ');disp('============================================== // ==============================================');disp(' ');
damp(ssr);
%%

a = 2;
b = 5;






%% END
rmpath(genpath(path));
