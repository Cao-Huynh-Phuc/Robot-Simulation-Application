function [theta1_real,theta2_real,d3_real,theta4_real] = noncentralize_control(theta1_d,theta2_d,d3_d,theta4_d,t,tf,Ts)
global theta1 theta2 d3 theta4
global DynamicModel
sig = [t;theta1_d];
save theta1_d sig
sig = [t;theta2_d];
save theta2_d sig
sig = [t;d3_d];
save d3_d sig
sig = [t;theta4_d];
save theta4_d sig

disp('loaded')
set_param(DynamicModel,'StopTime',num2str(tf));
set_param(DynamicModel,'SimulationMode','normal');
set_param(DynamicModel,'FixedStep',num2str(Ts));

set_param('Dynamic/DC_motor_1/Integrator','InitialCondition',num2str(theta1));
set_param('Dynamic/DC_motor_2/Integrator','InitialCondition',num2str(theta2));
set_param('Dynamic/DC_motor_3/Integrator','InitialCondition',num2str(d3));
set_param('Dynamic/DC_motor_4/Integrator','InitialCondition',num2str(theta4));
%set_param(Model,'SimulationCommand','start');
sim(DynamicModel);
%pause(10);
% while(get_param(Model,'SimulationStatus') ~= 'stopped')
%     
% end
load('theta1_noncentralize_control.mat');
load('theta2_noncentralize_control.mat');
load('d3_noncentralize_control.mat');
load('theta4_noncentralize_control.mat');
theta1_real = theta1_noncentralize_control(2,:);
theta2_real = theta2_noncentralize_control(2,:);
d3_real = d3_noncentralize_control(2,:);
theta4_real = theta4_noncentralize_control(2,:);
