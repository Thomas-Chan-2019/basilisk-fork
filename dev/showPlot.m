%% LATEX INITIALIZATION:
clear all; close all;
clc;
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

%% 
% dataPath = '/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultData/base_case_1_xr_index_1_plot_data.mat';
% dataPath = '/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultData/base_case_4_xyzr_index_1_plot_data.mat';
dataPath = '/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultData/control_case_7_xyzr_index_0_plot_data.mat';

data = load(dataPath);

simLength = data.simLength;
timeLineSetMin = data.timeLineSetMin;
dataUsReq = data.dataUsReq;
dataSigmaBN = data.dataSigmaBN;
dataOmegaRN_N = data.dataOmegaRN_N;
dataRW = data.dataRW;
dataThrust = data.dataThrust;
dataCmdForce = data.dataCmdForce;
dataOmegaRN_B = data.dataOmegaRN_B;
dr = data.dr; dr = permute(dr, [3 2 1]); % Switch back dimension for correct plotting.
oed = data.oed;

%%
% figure; plot3(dr(1,:,2),dr(2,:,2),dr(3,:,2));
% title('$x^2$ Trial');

t = timeLineSetMin;
figure; plot(t, dr(1,:,1), 'r', t, dr(2,:,1), 'g', t, dr(3,:,1), 'b');
xlabel('Time (min)'); ylabel('dr [m]');
legend('$x$','$y$','$z$');

figure; plot(t, dr(1,:,2), 'r', t, dr(2,:,2), 'g', t, dr(3,:,2), 'b');
xlabel('Time (min)'); ylabel('dr [m]');
legend('$x$','$y$','$z$');