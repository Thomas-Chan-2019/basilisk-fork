%% LATEX INITIALIZATION:
clear; 
close all;
clc;
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

set(groot,'DefaultFigureWindowStyle','docked'); % Dock all figures!

set(groot,'defaultAxesFontSize',40); 
set(groot, 'defaultLineLineWidth', 1.5);  % Axes line width

%%
% dataPath = '/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultData/base_case_1_xr/base_case_1_xr_plot_data.mat';
% dataPath = '/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultData/base_case_2_xyr/base_case_2_xyr_plot_data.mat';
% dataPath = '/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultData/base_case_3_zr_zrdot/base_case_3_zr_zrdot_plot_data.mat';
% dataPath = '/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultData/base_case_4_xyzr/base_case_4_xyzr_plot_data.mat';
% dataPath = '/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultData/control_case_5_yr/control_case_5_yr_plot_data.mat';
% dataPath = '/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultData/control_case_6_xyr/control_case_6_xyr_plot_data.mat';
% dataPath = '/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultData/control_case_7_xyzr/control_case_7_xyzr_plot_data.mat';

dataPath = '/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultData/init_config/init_config_plot_data.mat';

% Extract file name:
[~, export_case_name, ~] = fileparts(dataPath); 
export_plots_path = "/home/thomas/Documents/gits/basilisk-fork/dev/MultiSatBskSim/ResultPlotsMATLAB/" + export_case_name + "/";


%%
% Example of calling the function
% LoadCombine_mat(dataPath);
load(dataPath);
controllerOn = 0;
% controllerOn = 1;

%% Plots:

% Gather data:
t = timeLineSetMin(1,:);
numSC = length(simLength);
sim_dim = simLength(1);


for i=1:numSC

% 0) Title text & indexing:
title_text_base = "Spacecraft Index " + num2str(i-1) + " - ";

% 1) Relative Position - Hill Frame x-y-z positions from Target S/C:
rel_pos = squeeze(dr_index(i,:,:)); % MATLAB `squeeze()` to reduce a dimension!

fig(1) = figure('Name','rel_pos'); % apply_custom_style();
subplot(3,1,1); plot(t, rel_pos(:,1));
title(title_text_base + "Relative Position from Target S/C (Hill-frame)");
xlabel('Time $t \ [min]$'); 
ylabel('$x \ [m]$');
subplot(3,1,2); plot(t, rel_pos(:,2));
xlabel('Time $t \ [min]$'); 
ylabel('$y \ [m]$');
subplot(3,1,3); plot(t, rel_pos(:,3));
xlabel('Time $t \ [min]$'); 
ylabel('$z \ [m]$');

% 2) MRP Pointing Errors + Angular Velocity (Body w.r.t. Hill frame):
sigma_BH = squeeze(dataSigmaBR(i,:,:)); % MATLAB `squeeze()` to reduce a dimension!
omega_BH = squeeze(dataOmegaBR(i,:,:)); % MATLAB `squeeze()` to reduce a dimension!

fig(2) = figure('Name','sigma_BH_omega_BH'); % apply_custom_style();
subplot(2,1,1); plot(t, sigma_BH);
xlabel('Time $t \ [min]$'); 
ylabel('$\sigma_{B/H} \ [rad]$');
legend('$\sigma_{1}$', '$\sigma_{2}$', '$\sigma_{3}$');
title(title_text_base + "MRP Pointing Error - Body w.r.t. Hill-frame");

subplot(2,1,2); plot(t, omega_BH);
xlabel('Time $t \ [min]$'); 
ylabel('$\omega_{B/H} \ [rad/s]$');
legend('$\omega_{x}$', '$\omega_{y}$', '$\omega_{z}$');
title(title_text_base + "Angular Velocity - Body w.r.t. Hill-frame");

if controllerOn
% 3) Cmd Force:
    F_cmd = squeeze(dataCmdForce(i,:,:)); % MATLAB `squeeze()` to reduce a dimension!
    
    fig(3) = figure('Name','F_cmd'); % apply_custom_style();
    plot(t,F_cmd); 
    xlabel('Time $t \ [min]$'); 
    ylabel('Commanded Force $F_{cmd} \ [N]$');
    title(title_text_base + "Commanded Control Force");
    legend('$F_x$','$F_y$','$F_z$');
    
% 4) Cmd Torque:
    L_cmd = squeeze(dataUsReq(i,:,:)); % MATLAB `squeeze()` to reduce a dimension!
    
    fig(4) = figure('Name','L_cmd'); % apply_custom_style();
    plot(t,F_cmd); 
    xlabel('Time $t \ [min]$'); 
    ylabel('Commanded Torque $L_{cmd} \ [Nm]$');
    title(title_text_base + "Commanded Control Torque");
    legend('$L_x$','$L_y$','$L_z$');
    
% 5) Thruster Actuations:
    % Total Impulse P_tot = summation(F*dt), dt = sampling time*60 (minute):
    F_thrusters = squeeze(dataThrust(i,:,:)); % MATLAB `squeeze()` to reduce a dimension!
    dt = (t(2) - t(1)) * 60; % Min -> Second
    F_tot = sum(sum(F_thrusters)); % Discrete sum of forces
    P_tot = F_tot * dt
    
    fig(5) = figure('Name','F_thrusters'); % apply_custom_style();
    plot(t,F_thrusters); 
    xlabel('Time $t \ [min]$'); 
    ylabel('Actuated Force $F_{thrusters} \ [N]$');
    title(title_text_base + "Thrusters Actuated Force");
    legend('$+x$','$-x$','$+y$','$-y$','$+z$','$-z$'); % Currently hard-coded for 6 thrusters.
    
    P_tot_txt = "Total Impulse: $P_{tot} = $ " + num2str(P_tot) + " $Ns$";
    text(t(end)/2,max(max(F_thrusters))/2,P_tot_txt,'HorizontalAlignment','center', 'FontSize',36);
    
% 6) RW Actuations (motorTorque):
    L_RW = squeeze(dataRW(i,:,:)); % MATLAB `squeeze()` to reduce a dimension!
    
    fig(6) = figure('Name','L_RW'); % apply_custom_style();
    plot(t,L_RW); 
    title(title_text_base + "RW Actuated Torque");
    xlabel('Time $t \ [min]$'); 
    ylabel('Actuated Torque $L_{RW} \ [Nm]$');
    legend('RW $x$','RW $y$','RW $z$'); % Currently hard-coded for 3 RWs.
end

% End loop:
    % savefig(fig_index_array);
    % exportFigToPath(export_plots_path, numSC-1, export_case_name, fig_index_array);
    exportFigToPath(export_plots_path, i-1, fig);
    for n=1:length(fig)
        figure(fig(n)); 
        exportPlotToPath(export_plots_path, i-1, fig(n).Name);
    end
    close all;
end

% 7) Plot orbits (out of for loop):
% figure;
% xlabel('Time $t \ [min]$'); 
% ylabel('Actuated Force $F_{thrusters} \ [N]$');
% 
% title("Formation Flying Orbits");

%% Functions:
function apply_custom_style()
    % Set global figure and axes properties
    set(gcf, 'FontSize', 16);  % Axes font size
    set(gcf, 'LineWidth', 1.5);  % Axes line width
    % set(gcf, 'Position', [100, 100, 800, 600]);  % Figure size
end

function exportPlotToPath(export_path_base, SCIndex, filename)
    export_path = export_path_base + "index_" + num2str(SCIndex);
    if ~exist(export_path, 'dir')
    % If the folder does not exist, create it
        mkdir(export_path);
        disp(['Created export folder: ' export_path]);
    end 
    
    % Assuming everytime `figure;` is called in every plot
    % apply_custom_style();

    % Save the plots into .eps and .fig:
    savePlotFile = export_path + "/" + filename + ".eps";
    exportgraphics(gcf, savePlotFile, 'Resolution', 300);

    % Close the figure upon saving:
    % close all;
end

% Save an array of figures `fig` into ONE single .fig file:
function exportFigToPath(export_path_base, SCIndex, fig)
    export_path = export_path_base + "index_" + num2str(SCIndex);
    if ~exist(export_path, 'dir')
    % If the folder does not exist, create it
        mkdir(export_path);
        disp(['Created export folder: ' export_path]);
    end 
    
    % Assuming everytime `figure;` is called in every plot
    for i=1:length(fig)
        fig(i); % apply_custom_style();
    end

    % Save the plots into .eps and .fig:
    savePlotFile = export_path + "/" + "index_" + num2str(SCIndex) + ".fig"
    savefig(fig, savePlotFile);
    % close(fig);
    % Close the figure upon saving:
    % close all;
end

function LoadCombine_mat(folder_path)
    % folder_path: Path to the folder containing the .mat files (files are named index_0_plot_data.mat, index_1_plot_data.mat, etc.).

    % Find all .mat files in the folder matching the pattern 'index_*_plot_data.mat'
    mat_files = dir(fullfile(folder_path, 'index_*_plot_data.mat'));
    
    % Check if there are any files
    if isempty(mat_files)
        error('No .mat files found in the specified folder with the pattern index_*_plot_data.mat');
    end
    
    % Define an empty structure to hold the combined variables
    combined_variables = struct();
    
    % Loop over each file found
    for i = 1:length(mat_files)
        % Create the full path to the file
        filename = fullfile(folder_path, mat_files(i).name);
        
        % Load the .mat file
        data = load(filename);
        
        % Loop over each variable in the loaded data
        var_names = fieldnames(data);
        for j = 1:length(var_names)
            var_name = var_names{j};
            var_value = data.(var_name);
            
            % If this variable has already been initialized, concatenate along a new dimension
            if isfield(combined_variables, var_name)
                combined_variables.(var_name) = cat(ndims(combined_variables.(var_name)) + 1, combined_variables.(var_name), var_value);
            else
                % Initialize the variable with the current value from the file
                combined_variables.(var_name) = var_value;
            end
        end
    end
    
    % Load the combined variables directly into the MATLAB workspace
    var_names_combined = fieldnames(combined_variables);
    for k = 1:length(var_names_combined)
        assignin('base', var_names_combined{k}, combined_variables.(var_names_combined{k}));
    end

    % Save the combined variables into a single .mat file for plotting or further analysis
    % save('combined_data.mat', '-struct', 'combined_variables');
    
    % % Example plot: Assuming one of the variables is 'data1'
    % if isfield(combined_variables, 'data1')
    %     figure;
    %     % Here, you may want to adjust plotting according to the dimensionality
    %     plot(squeeze(combined_variables.data1));
    %     title('Plot of Combined data1 Variable');
    % else
    %     disp('No variable named ''data1'' found.');
    % end
end

