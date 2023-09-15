%   TITLE     : ROBUST ADAPTIVE COMPENSATION OF FBRTHS TESTING WITH UNCERTAIN COMPLIANCE SPRING AND FORCE MEASUREMENT NOISE
%   SUBJECT   : INPUTS - REFERENCE STRUCTURE
%   AUTHOR    : DIEGO ARAYA IGLESIAS
%% INITIALIZING
clc;clear;close all;format shortG;
path = 'C:\MATLAB\THESIS_MATLAB_CODE';
addpath(genpath(path));
%% REFERENCE STRUCTURE
% DEGREE OF FREEDOMS
mr     = [98.3 98.3 98.3];
kr_ini = [0.516 0.684 0.684]*1e6;
ndofr  = length(mr);
iotar  = diag(eye(ndofr));
% % WALL STIFFNESS
kw      = 20*kr_ini(1);
kRatio  = kw/kr_ini(1);
kr      = kr_ini;
kr(1)   = kr(1)+kw; 
% % PHILLIPS: MULTI-ACTUATOR NONLINEAR NUMERICAL STUDY (EXPERIMENTAL SUBSTRUCTURE)
Mr = 98.3*eye(ndofr);
Kr_ini = K_matrix_ensamble(kr_ini);
Kr = K_matrix_ensamble(kr);

% FUNDAMENTAL PERIODS & FREQUENCIES
[Phi,wn2] = eig(Kr_ini,Mr,'vector');
fn        = sqrt(wn2)./2./pi;
Tn        = 1./fn;
% FUNDAMENTAL PERIODS & FREQUENCIES
[Phiw,wn2w] = eig(Kr,Mr,'vector');
fnw         = sqrt(wn2w)./2./pi;
Tnw         = 1./fnw;

% DAMPING MATRIX FROM PAPER
% Cr = [175 -50 0;-50 100 -50;0 -50 50];

% RAYLEIGH DAMPING
% zr = [5;5]/100;
% Cr = rayleigh_damping(Mr,Kr,zr);

% MODAL DAMPING
% zr = 1/100*[0.31;0.62;0.63];
zr = 1/100*[5;5;5];
Cr = modal_damping(Mr,Kr,zr);

% STATE-SPACE FORM
ssAr = [zeros(ndofr),eye(ndofr);-Mr\Kr,-Mr\Cr];
ssBr = [zeros(ndofr,1);-(Mr\Mr)*iotar];
ssCr = [eye(ndofr),zeros(ndofr);-Kr,-Cr];
ssDr = [zeros(ndofr,1);zeros(ndofr,1)];
ssr  = ss(ssAr,ssBr,ssCr,ssDr);
disp(' ');disp('============================================== // ==============================================');disp(' ');
damp(ssr);
% save('TMC01_MF01_REFERENCE_STRUCTURE.mat','mr','kr','zr','ndofr','iotar','Mr','Kr','Cr','ssAr','ssBr','ssCr','ssDr','kw');


% TABLE WITH RESULTS
rowLabelst = {'Mode 1';'Mode 2';'Mode 3'};
varNamest  = {'Tn [s]','fn [Hz]'};
disp(' ');disp('----------------------------- o ------------------------------');disp(' ');
disp('<strong> TABLE 1. FUNDAMENTAL PERIODS & FREQUENCIES OF REFERENCE STRUCTURE WITHOUT WALL</strong>');disp(' ');
Table = table(Tn,fn,'VariableNames',varNamest,'RowNames',rowLabelst);
disp(Table);
disp(' ');disp('----------------------------- o ------------------------------');disp(' ');
% TABLE WITH RESULTS
rowLabelst = {'Mode 1';'Mode 2';'Mode 3'};
varNamest  = {'Tn [s]','fn [Hz]'};
disp(' ');disp('----------------------------- o ------------------------------');disp(' ');
disp('<strong> TABLE2. FUNDAMENTAL PERIODS & FREQUENCIES OF REFERENCE STRUCTURE WITH WALL </strong>');disp(' ');
Table = table(Tnw,fnw,'VariableNames',varNamest,'RowNames',rowLabelst);
disp(Table);
disp(' ');disp('----------------------------- o ------------------------------');disp(' ');
% SAVE Mr,Cr & Kr MATRICES IN A .mat FILE
save('ReferenceStructure.mat','Mr','Cr','Kr','kw');


%% PLOTS
tLim = 40;
% PLOT OPTIONS
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaulttextinterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
poptions.fontsz = 15;
poptions.ticksz = poptions.fontsz-2;
% PLOT 1: VIBRATION MODES
figure('Position', [600 200 800 700])
TL1 = tiledlayout(1,ndofr);
ylabel(TL1,'\textbf{Story}','fontsize',poptions.fontsz,'interpreter','latex')
for j = 1:ndofr
    ax1 = nexttile;
    hold(ax1,"on");
    grid(ax1,"on");
    ax1.XColor = 'k';
    ax1.YColor = 'k';
    ax1.FontSize = poptions.ticksz;
    ax1.FontWeight = 'bold';
    set(gcf,'color','w');
    box on;
    xticks(0);
    yticks([0 1 2 3]);
    yticklabels(ax1,{'0','1','2','3'});
    xlim(ax1,[-1*abs(max([Phi,Phiw],[],'all','ComparisonMethod','abs'))*1.1,abs(max([Phi,Phiw],[],'all','ComparisonMethod','abs')*1.1)]);
    title(ax1,{['\textbf{Mode} $\mathbf{',num2str(j),'}$']},'interpreter','latex');
    p(1) = plot([0;Phi(:,j)],0:1:ndofr,'-c','LineWidth',2,'DisplayName','\textbf{Without wall}');
    p(2) = plot(Phi(:,j),1:1:ndofr,'ok','LineWidth',1.8,'MarkerSize',8,'MarkerFaceColor','c');
    p(3) = plot([0;Phiw(:,j)],0:1:ndofr,'-m','LineWidth',2,'DisplayName','\textbf{With wall}');
    p(4) = plot(Phiw(:,j),1:1:ndofr,'ok','LineWidth',1.8,'MarkerSize',8,'MarkerFaceColor','m');
end
leg=legend([p(1) p(3)],'Orientation', 'Horizontal');
leg.Layout.Tile = 'North';

%% END
rmpath(genpath(path));