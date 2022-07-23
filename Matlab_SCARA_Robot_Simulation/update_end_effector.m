function [p4,o4] = update_end_effector(theta1, theta2, d3, theta4, handles)
[Az,El] = view;
%% Update POSE
% Get parameters
global a1 a2 d1 d4
global disable_kinematics_singularity
if (isempty(a1))
    a1 = 0.45;
    a2 = 0.4;
    d1 = 0;
    d4 = -0.06;
end
global mode_RobotView
global value_3dRobotView_opacity
%syms a1 a2 d1 d3 d4 theta1 theta2 theta4
a     = [a1    ; a2     ;  0  ; 0     ];
alpha = [0     ; 0      ;  0  ; pi    ];
d     = [d1    ; 0      ;  d3 ; d4    ];
theta = [theta1; theta2 ;  0  ; theta4];

%% FK Matrix
A0_0 = eye(4);
A1_0 = Link_matrix(a(1),alpha(1),d(1),theta(1)) ;
A2_1 = Link_matrix(a(2),alpha(2),d(2),theta(2)) ;
A3_2 = Link_matrix(a(3),alpha(3),d(3),theta(3)) ;
A4_3 = Link_matrix(a(4),alpha(4),d(4),theta(4)) ;

% A2_0=A2_1*A1_0;
% A3_0=A3_2*A2_1*A1_0;
% A4_0=A4_3*A3_2*A2_1*A1_0;   % Teb
A2_0=A1_0*A2_1;
A3_0=A1_0*A2_1*A3_2;
A4_0=A1_0*A2_1*A3_2*A4_3;   % Teb

%%
z0 = [0 0 1]';
p0 = [0 0 0]';
z1 = A1_0(1:3,3);
p1 = A1_0(1:3,4);
z2 = A2_0(1:3,3);
p2 = A2_0(1:3,4);
z3 = A3_0(1:3,3);
p3 = A3_0(1:3,4);
z4 = A4_0(1:3,3);
p4 = A4_0(1:3,4);

J = [cross(z0,(p4-p0)) cross(z1,(p4-p1))     z2     cross(z3,(p4-p3));...
           z0                z1          zeros(3,1)       z3];

%% Update
% A1_0=eval(A1_0);
% A2_0=eval(A2_0);
% A3_0=eval(A3_0);
% A4_0=eval(A4_0);
%if (rank(J)==4)
if ((abs(det(J([1 2 3 6],:)))>1e-4) || (disable_kinematics_singularity == true))
    set(handles.slider_theta1,'value',theta1/pi*180);
    set(handles.slider_theta2,'value',theta2/pi*180);
    set(handles.slider_d3,'value',-d3);
    set(handles.slider_theta4,'value',theta4/pi*180);
    set(handles.edit_theta1,'string',round(theta1/pi*180,3));
    set(handles.edit_theta2,'string',round(theta2/pi*180,3));
    set(handles.edit_d3,'string',round(-d3,3));
    set(handles.edit_theta4,'string',round(theta4/pi*180,3));
    
p0 = [0;0;0];
[p1, o1] = cal_pose(A1_0,p0);
[p2, o2] = cal_pose(A2_0,p0);
[p3, o3] = cal_pose(A3_0,p0);
[p4, o4] = cal_pose(A4_0,p0);

set(handles.edit_x,'String',p4(1));
set(handles.edit_y,'String',p4(2));
set(handles.edit_z,'String',p4(3));
set(handles.edit_roll,'String',o4(1)*180/pi);
set(handles.edit_pitch,'String',o4(2)*180/pi);
set(handles.edit_yaw,'String',o4(3)*180/pi);
%% Plot
%cla reset
cla
%hold on
%grid on

Con =[p0,p1,p2,p3,p4];


%base
t = (1/16:1/8:1)'*2*pi;
x = cos(t);
y = sin(t);
z = 0*[1 1 1 1 1 1 1 1]';
fill3(handles.axes_robot,x,y,z,'y','FaceAlpha',0.25)

%%
if (mode_RobotView == 0)
    % define links
    p4_picker = [p4(1)-(p4(1)-p3(1))/2;p4(2)-(p4(2)-p3(2))/2;p4(3)+0.05];
    line1=[[p0(1) p1(1)];[p0(2) p1(2)];[p0(3)+d(1) p1(3)]];
    line2=[[p1(1) p2(1)];[p1(2) p2(2)];[p1(3) p2(3)]];
    line3=[[p2(1) p3(1)];[p2(2) p3(2)];[p2(3) p3(3)]];
    line4=[[p3(1) p4_picker(1)];[p3(2) p4_picker(2)];[p3(3) p4_picker(3)]];
    
    base_plot=plot3(handles.axes_robot,[0 0],[ 0 0 ],[0 d(1)],'linewidth',15,'color', 'black');
    base_plot.Color(4)=1;
    plot3(handles.axes_robot,0,0,-0.363,'rx','linewidth',5)
    %link1
    line1_plot=plot3(handles.axes_robot,line1(1,:),line1(2,:),line1(3,:),'linewidth',10,'color', 'magenta');
    line1_plot.Color(4)=0.5;
    %link2
    line2_plot=plot3(handles.axes_robot,line2(1,:),line2(2,:),line2(3,:),'linewidth',5,'color', 'green');
    line2_plot.Color(4)=0.5;
    %link3
    line3_plot=plot3(handles.axes_robot,line3(1,:),line3(2,:),line3(3,:),'linewidth',5,'color', [0.6350 0.0780 0.1840]);
    line3_plot.Color(4)=0.5;
    %link4
    line4_plot=plot3(handles.axes_robot,line4(1,:),line4(2,:),line4(3,:),'linewidth',5,'color', 'blue');
    line4_plot.Color(4)=0.5;
    

    %picker arm
    picker(p4_picker(1),p4_picker(2),p4_picker(3),round(o4(3),3),0.15,abs(d4)/2);

    %rotate3d on
    % view(45,45)
else
    % define links
    line1=[[p0(1) p1(1)];[p0(2) p1(2)];[p0(3) p1(3)]];
    line2=[[p1(1) p2(1)];[p1(2) p2(2)];[p1(3) p2(3)]];    
    % Ve base
    color_base = [28 188 180]/255;
    plot_cube_rectangular(0,0,0,0.24,0.24,0.02,color_base,value_3dRobotView_opacity)
    plot_cube_cylinder(0,0,0.02,0.12,0.7457*d(1),color_base,value_3dRobotView_opacity)
    plot_cube_cylinder(line1(1,1),line1(2,1),0.7826*d(1),0.07,0.0185*d(1)*2+0.08,color_base,value_3dRobotView_opacity);
    % 
    % %link 1
    color_link_1 = [80 215 112]/255;
    plot_cube_cylinder(line1(1,1),line1(2,1),0.7826*d(1),0.1,0.0185*d(1),color_link_1,value_3dRobotView_opacity);
    plot_cube_cylinder(line1(1,2),line1(2,2),0.7826*d(1),0.1,0.0185*d(1),color_link_1,value_3dRobotView_opacity);
    plot_cube_cylinder(line1(1,2),line1(2,2),0.8076*d(1),0.1,0.08,color_link_1,value_3dRobotView_opacity);
    plot_cube_cylinder(line1(1,1),line1(2,1),0.8076*d(1),0.1,0.08,color_link_1,value_3dRobotView_opacity);

    [pp1,pp2]=find_common_tangent([line1(1,1) line1(2,1)],[line1(1,2) line1(2,2)],0.1,0.1,1,-1);
    [pp4,pp3]=find_common_tangent([line1(1,1) line1(2,1)],[line1(1,2) line1(2,2)],0.1,0.1,1,1);

    fill3([pp1(1) pp2(1) pp3(1) pp4(1)],[pp1(2) pp2(2) pp3(2) pp4(2)],[0.8076*d(1) 0.8076*d(1) 0.8076*d(1) 0.8076*d(1)],color_link_1,'FaceAlpha',value_3dRobotView_opacity)
    fill3([pp1(1) pp2(1) pp3(1) pp4(1)],[pp1(2) pp2(2) pp3(2) pp4(2)],[0.9815*d(1) 0.9815*d(1) 0.9815*d(1) 0.9815*d(1)],color_link_1,'FaceAlpha',value_3dRobotView_opacity)
    fill3([pp1(1) pp4(1) pp4(1) pp1(1)],[pp1(2) pp4(2) pp4(2) pp1(2)],[0.8076*d(1) 0.8076*d(1) 0.9815*d(1) 0.9815*d(1)],color_link_1,'FaceAlpha',value_3dRobotView_opacity)
    fill3([pp3(1) pp4(1) pp4(1) pp3(1)],[pp3(2) pp4(2) pp4(2) pp3(2)],[0.8076*d(1) 0.8076*d(1) 0.9815*d(1) 0.9815*d(1)],color_link_1,'FaceAlpha',value_3dRobotView_opacity)
    fill3([pp2(1) pp3(1) pp3(1) pp2(1)],[pp2(2) pp3(2) pp3(2) pp2(2)],[0.8076*d(1) 0.8076*d(1) 0.9815*d(1) 0.9815*d(1)],color_link_1,'FaceAlpha',value_3dRobotView_opacity)
    fill3([pp1(1) pp2(1) pp2(1) pp1(1)],[pp1(2) pp2(2) pp2(2) pp1(2)],[0.8076*d(1) 0.8076*d(1) 0.9815*d(1) 0.9815*d(1)],color_link_1,'FaceAlpha',value_3dRobotView_opacity)
    
    % 
    %link2
    color_link_2 = [168 228 252]/255;
    plot_cube_cylinder(line1(1,2),line1(2,2),0.9815*d(1),0.11,0.0185*d(1),color_link_2,value_3dRobotView_opacity);
    plot_cube_cylinder(line1(1,2),line1(2,2),0.7641*d(1),0.09,0.0175*d(1),color_link_2,value_3dRobotView_opacity);
    plot_cube_cylinder(line2(1,1),line2(2,1),d(1),0.08,0.286,color_link_2,value_3dRobotView_opacity)
    plot_cube_cylinder(line2(1,1),line2(2,1),0.7826*d(1),0.08,0.28*0.286+0.0185*d(1),color_link_2,value_3dRobotView_opacity)
    plot_cube_cylinder(line2(1,2),line2(2,2),d(1),0.08,0.28*0.286,color_link_2,value_3dRobotView_opacity)

    [pp1,pp2]=find_common_tangent([line2(1,1) line2(2,1)],[line2(1,2) line2(2,2)],0.08,0.08,1,-1);
    [pp4,pp3]=find_common_tangent([line2(1,1) line2(2,1)],[line2(1,2) line2(2,2)],0.08,0.08,1,1);

    temp_1 = 0.28*0.286;
    temp_2 = 0.286;
    fill3([pp1(1) pp2(1) pp3(1) pp4(1)],[pp1(2) pp2(2) pp3(2) pp4(2)],[d(1) d(1) d(1) d(1)],color_link_2,'FaceAlpha',value_3dRobotView_opacity)
    fill3([pp1(1) pp2(1) pp3(1) pp4(1)],[pp1(2) pp2(2) pp3(2) pp4(2)],[d(1)+temp_1 d(1)+temp_1 d(1)+temp_1 d(1)+temp_1],color_link_2,'FaceAlpha',value_3dRobotView_opacity)
    fill3([pp1(1) pp4(1) pp4(1) pp1(1)],[pp1(2) pp4(2) pp4(2) pp1(2)],[d(1) d(1) d(1)+temp_1 d(1)+temp_1],color_link_2,'FaceAlpha',value_3dRobotView_opacity)
    fill3([pp3(1) pp4(1) pp4(1) pp3(1)],[pp3(2) pp4(2) pp4(2) pp3(2)],[d(1) d(1) d(1)+temp_1 d(1)+temp_1],color_link_2,'FaceAlpha',value_3dRobotView_opacity)
    fill3([pp2(1) pp3(1) pp3(1) pp2(1)],[pp2(2) pp3(2) pp3(2) pp2(2)],[d(1) d(1) d(1)+temp_1 d(1)+temp_1],color_link_2,'FaceAlpha',value_3dRobotView_opacity)
    fill3([pp1(1) pp2(1) pp2(1) pp1(1)],[pp1(2) pp2(2) pp2(2) pp1(2)],[d(1) d(1) d(1)+temp_1 d(1)+temp_1],color_link_2,'FaceAlpha',value_3dRobotView_opacity)

    % pp5 = (pp1+pp2)/2;
    % pp6 = (pp3+pp4)/2;
    pp5 = pp1 + (pp2-pp1)*1/3.8;
    pp6 = pp3 + (pp4-pp3)*2.8/3.8;
    fill3([pp1(1) pp5(1) pp5(1) pp1(1)],[pp1(2) pp5(2) pp5(2) pp1(2)],[d(1)+temp_1 d(1)+temp_1 d(1)+temp_2 d(1)+temp_2],color_link_2,'FaceAlpha',value_3dRobotView_opacity)
    fill3([pp4(1) pp6(1) pp6(1) pp4(1)],[pp4(2) pp6(2) pp6(2) pp4(2)],[d(1)+temp_1 d(1)+temp_1 d(1)+temp_2 d(1)+temp_2],color_link_2,'FaceAlpha',value_3dRobotView_opacity)
    fill3([pp1(1) pp5(1) pp6(1) pp4(1)],[pp1(2) pp5(2) pp6(2) pp4(2)],[d(1)+temp_2 d(1)+temp_2 d(1)+temp_2 d(1)+temp_2],color_link_2,'FaceAlpha',value_3dRobotView_opacity)
    fill3([pp5(1) pp2(1) pp5(1)],[pp5(2) pp2(2) pp5(2)],[d(1)+temp_1 d(1)+temp_1 d(1)+temp_2],color_link_2,'FaceAlpha',value_3dRobotView_opacity);
    fill3([pp6(1) pp3(1) pp6(1)],[pp6(2) pp3(2) pp6(2)],[d(1)+temp_1 d(1)+temp_1 d(1)+temp_2],color_link_2,'FaceAlpha',value_3dRobotView_opacity);
    fill3([pp5(1) pp2(1) pp3(1) pp6(1)],[pp5(2) pp2(2) pp3(2) pp6(2)],[d(1)+temp_2 d(1)+temp_1 d(1)+temp_1 d(1)+temp_2],color_link_2,'FaceAlpha',value_3dRobotView_opacity)


    % %link3
    color_link_3_1 = [98 120 177]/255;
    color_link_3_2 = [0.4660 0.6740 0.1880];
    plot_cube_cylinder(p2(1),p2(2),d(1)-0.02,0.05,0.02,color_link_3_2,value_3dRobotView_opacity)
    plot_cube_cylinder(p2(1),p2(2),d(1)-0.035,0.02,0.015,color_link_3_2,value_3dRobotView_opacity)
    plot_cube_cylinder(p3(1),p3(2),p3(3)-0.035,0.02,0.572,color_link_3_1,value_3dRobotView_opacity);
    plot_cube_cylinder(p3(1),p3(2),p3(3)+0.572-0.035,0.03,0.03,color_link_3_1,value_3dRobotView_opacity);
end

%%
% Plot coordination
    plot_coordinate(p0(1),p0(2),p0(3),A0_0,0,0.15);
    plot_coordinate(p1(1),p1(2),p1(3),A1_0,1,0.15);
    plot_coordinate(p2(1),p2(2),p2(3),A2_0,2,0.15);
    plot_coordinate(p3(1),p3(2),p3(3),A3_0,3,0.15);
    plot_coordinate(p4(1),p4(2),p4(3),A4_0,4,0.15);
% picker arm
% for i=0:pi/4:3*pi/4
%     picker(p4_picker(1),p4_picker(2),p4_picker(3),o4(3)*pi/180+i,0.15,0.05);
% end
p4_picker = [p4(1)-(p4(1)-p3(1))/2;p4(2)-(p4(2)-p3(2))/2;p4(3)+0.05];
picker(p4_picker(1),p4_picker(2),p4_picker(3),round(o4(3),3),0.15,0.05);

%%
plot3(handles.axes_robot,Con(1,:),Con(2,:),Con(3,:),'rx','linewidth',5)
plot_workspace(handles);
view(Az,El)
%IsKinematicsSingularity = false;
else
    %IsKinematicsSingularity = true;
    warndlg('Kinematics Singularity! You shoudnt go to this point','Warning');
end
