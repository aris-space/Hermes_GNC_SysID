%% Clean UP

close all; clear all; clc;

%% create filtered data
%%lowpass_yaw_derivatives;
clearvars -except name     %deletes all variables except name in workspace

%% read data
df = readtable('processedData.csv');
%df.yaw = unwrap(df.yaw); % for angles

figure()
plot(df.yaw)

start_step = 1;
end_step = size(df,1);
iter = 1;

sysid_data = iddata([df.x_dot_B(start_step:iter:end_step), df.rca(start_step:iter:end_step), df.z_dot_B(start_step:iter:end_step), df.roll(start_step:iter:end_step)],...
    [df.delta_asym(start_step:iter:end_step), df.delta_sym(start_step:iter:end_step)], mean(df.diff_time(start_step)));

sysid_data.OutputName = {'x dot B';'course angle';'z dot B';'roll'};
sysid_data.OutputUnit = {'m/s';'rad';'m/s';'rad'};


sysid_data.InputName = {'delta_asym';'delta_sym'}; %delta_bar
sysid_data.InputUnit = {'';''};
%% 4 DOF parameters


% from first to last element: C_L_0, C_L_SYM_DEFLECTION,
% C_D_0, C_D_SYM_DEFLECTION, K_ROLL, T_ROLL  
Parameters.C_L_0 = 0.4360;
Parameters.C_L_SYM_DEFLECTION = 0.0065;
Parameters.C_D_0 = 0.2034;
Parameters.C_D_SYM_DEFLECTION = 0.0000;
Parameters.T_ROLL = 1.7203;
Parameters.K_ROLL = 0.5871;


order = [4 2 4];  % outputs, inputs, states


parameters = {Parameters.C_L_0, Parameters.C_L_SYM_DEFLECTION, Parameters.C_D_0,Parameters.C_D_SYM_DEFLECTION,...
    Parameters.T_ROLL, Parameters.K_ROLL};

initial_states = [df.x_dot_B(start_step); (df.rca(start_step)); df.z_dot_B(start_step); df.roll(start_step)]; %x_dot_rca and z_dot_rca

Ts = 0;

nonlinear_four_dof_model = idnlgrey('FourDOF_ODE_C_MEX', order, parameters, initial_states, Ts);
nonlinear_four_dof_model.Algorithm.SimulationOptions.Solver = 'ode15s';
%% define properties

% states x = [x_dot_B, yaw, z_dot_B, roll]

% give unit and name to initial states
nonlinear_four_dof_model.InitialStates(1).Name = 'x dot B';
nonlinear_four_dof_model.InitialStates(1).Unit = 'm/s';
nonlinear_four_dof_model.InitialStates(2).Name = 'course angle';
nonlinear_four_dof_model.InitialStates(2).Unit = 'rad';
nonlinear_four_dof_model.InitialStates(3).Name = 'z_dot_B';
nonlinear_four_dof_model.InitialStates(3).Unit = 'm/s';
nonlinear_four_dof_model.InitialStates(4).Name = 'roll';
nonlinear_four_dof_model.InitialStates(4).Unit = 'rad';

% give name to inputs
nonlinear_four_dof_model.InputName{1} = 'delta_asym';
nonlinear_four_dof_model.InputUnit{1} = '';
nonlinear_four_dof_model.InputName{2} = 'delta_sym';
nonlinear_four_dof_model.InputUnit{2} = '';

% give unit and name to Output

nonlinear_four_dof_model.OutputName{1} = 'x dot B';
nonlinear_four_dof_model.OutputUnit{1} = 'm/s';
nonlinear_four_dof_model.OutputName{2} = 'course angle';
nonlinear_four_dof_model.OutputUnit{2} = 'rad';
nonlinear_four_dof_model.OutputName{3} = 'z dot B';
nonlinear_four_dof_model.OutputUnit{3} = 'm/s';
nonlinear_four_dof_model.OutputName{4} = 'roll';
nonlinear_four_dof_model.OutputUnit{4} = 'rad';


% give unit, name and range to Parameters
nonlinear_four_dof_model.Parameters(1).Name = 'C_L_0';
nonlinear_four_dof_model.Parameters(1).Unit = '';

nonlinear_four_dof_model.Parameters(2).Name = 'C_L_SYM_DEFLECTION';
nonlinear_four_dof_model.Parameters(2).Unit = '';

nonlinear_four_dof_model.Parameters(3).Name = 'C_D_0';
nonlinear_four_dof_model.Parameters(3).Unit = '';

nonlinear_four_dof_model.Parameters(4).Name = 'C_D_SYM_DEFLECTION';
nonlinear_four_dof_model.Parameters(4).Unit = '';

nonlinear_four_dof_model.Parameters(5).Name = 'T_ROLL';
nonlinear_four_dof_model.Parameters(5).Unit = 's';

nonlinear_four_dof_model.Parameters(6).Name = 'K_ROLL';
nonlinear_four_dof_model.Parameters(6).Unit = 'rad';

% give bounds to parameters

nonlinear_four_dof_model.Parameters(1).Minimum = 0;
nonlinear_four_dof_model.Parameters(1).Maximum = 1;
nonlinear_four_dof_model.Parameters(2).Minimum = 0;
nonlinear_four_dof_model.Parameters(2).Maximum = 1;
nonlinear_four_dof_model.Parameters(3).Minimum = 0;
nonlinear_four_dof_model.Parameters(3).Maximum = 1;
nonlinear_four_dof_model.Parameters(4).Minimum = 0;
nonlinear_four_dof_model.Parameters(4).Maximum = 1;
nonlinear_four_dof_model.Parameters(5).Minimum = 0;
nonlinear_four_dof_model.Parameters(5).Maximum = 2;
nonlinear_four_dof_model.Parameters(6).Minimum = 0;
nonlinear_four_dof_model.Parameters(6).Maximum = 2;


%setpar(nonlinear_four_dof_model, 'Fixed', {true true true true true true});
%setpar(nonlinear_four_dof_model, 'Fixed', {false false false false false false});

%% System Identification
opt = nlgreyestOptions('SearchMethod','auto');
%opt = nlgreyestOptions('SearchMethod','fmincon');
%opt.SearchOptions.Algorithm = 'interior-point';
opt.Display = 'Full';
opt.SearchOptions.MaxIterations = 10;
%opt.OutputWeight = 'noise';
%opt.OutputWeight = diag([1, 1, 1, 1000]);
nonlinear_four_dof_model = nlgreyest(sysid_data, nonlinear_four_dof_model, opt);

%% output parameters and compare 
%[nonlinear_b_est, dnonlinear_b_est] = getpvec(nonlinear_four_dof_model,'free')

parameters_obtained = nonlinear_four_dof_model.Report.Parameters.ParVector
options_used = nonlinear_four_dof_model.Report.OptionsUsed
termination = nonlinear_four_dof_model.Report.Termination
fit = nonlinear_four_dof_model.Report.Fit

%% plots
%{
figure()
pe(sysid_data,nonlinear_four_dof_model)
saveas(gcf, 'png/greybox_pe_four_dof_' + name + '.png')
saveas(gcf, 'pdf/greybox_pe_four_dof_' + name + '.pdf')

figure()
step(nonlinear_four_dof_model)
saveas(gcf, 'png/greybox_step_four_dof_' + name + '.png')
saveas(gcf, 'pdf/greybox_step_four_dof_' + name + '.pdf')

figure()
resid(sysid_data,nonlinear_four_dof_model)
saveas(gcf, 'png/greybox_resid_four_dof_' + name + '.png')
saveas(gcf, 'pdf/greybox_resid_four_dof_' + name + '.pdf')

figure()
compare(sysid_data,nonlinear_four_dof_model)
ylim([5,15])
saveas(gcf, 'png/greybox_results_four_dof_' + name + '.png')
saveas(gcf, 'pdf/greybox_results_four_dof_' + name + '.pdf')
%}
name1 = figure('position', [200, 100, 1300, 1000]);
compare(sysid_data,nonlinear_four_dof_model)
grid();
ylim([5,15])
set(name1,'Units','Inches');
pos = get(name1,'Position');
set(name1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(name1,'pdf/resized_image_four_dof_' + name1,'-dpdf','-r0')

%%
%{
results = figure(1);
set(results,'Units','Inches');
pos = get(results,'Position');
set(results,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(results,'resized_image_four_dof_' + name,'-dpdf','-r0')
%}