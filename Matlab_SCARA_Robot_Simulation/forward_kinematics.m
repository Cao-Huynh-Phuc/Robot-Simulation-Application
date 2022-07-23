function forward_kinematics(handles)
global theta1 theta2 d3 theta4;
global disable_kinematics_singularity
%% Update POSE
% Get parameters

theta1_new = get(handles.slider_theta1,'value')/180*pi;
theta2_new = get(handles.slider_theta2,'value')/180*pi;
d3_new = -get(handles.slider_d3,'value');
theta4_new = get(handles.slider_theta4,'value')/180*pi;

J = jacobian_matrix(theta1_new, theta2_new, d3_new, theta4_new);
if ((abs(det(J([1 2 3 6],:)))>1e-4) || (disable_kinematics_singularity == true))
    theta1 = theta1_new;
    theta2 = theta2_new;
    d3 = d3_new;
    theta4 = theta4_new;
else   
    waitfor(warndlg('Kinematics Singularity!','Warning'));
end
update_end_effector(theta1, theta2, d3, theta4, handles);

