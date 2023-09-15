%   TITLE     : ROBUST ADAPTIVE COMPENSATION OF FBRTHS TESTING WITH UNCERTAIN COMPLIANCE SPRING AND FORCE MEASUREMENT NOISE
%   SUBJECT   : INPUTS - REFERENCE STRUCTURE
%   AUTHOR    : DIEGO ARAYA IGLESIAS
%% INITIALIZING
clc;clear;close all;format shortG;
path = 'C:\MATLAB\THESIS_MATLAB_CODE';
addpath(genpath(path));
%% SENSORS
sampling_frequency = 4096;
sampling_period    = 1/sampling_frequency;
% DISPLACEMENT
rmsNoiseDisp = 6.25e-14/6; % Noise power
rmsDisp      = sqrt(rmsNoiseDisp/sampling_period); % rms [mm]
% FORCE
rmsNoiseF     = 1.16e-3; % Noise power
rmsF          = sqrt(rmsNoiseF/sampling_period); % rms [N]
% SATURATION LIMITS
sat_limit_upper = +3.8; % Volts
sat_limit_lower = -3.8; % Volts
% QUANTIZATION INTERVAL
quantize_int = 1/2^18; % 18 bit channel
% SATURATION BLOCK
max_amb = [inf,inf,inf,inf];
min_amb = [0,0,0,0];
% SAVING RESULTS
clear sampling_frequency sampling_period
save('TMC01_MF04_SENSORS.mat');
%% END
rmpath(genpath(path));
clear