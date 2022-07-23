function update(handles)
syms a1 a2 d1 d3 d4 theta1 theta2 theta4
a     = [a1    ; a2     ;  0  ; 0     ];
alpha = [0     ; 0      ;  0  ; pi    ];
d     = [d1    ; 0      ;  d3 ; d4    ];
theta = [theta1; theta2 ;  0  ; theta4];

%% FK Matrix
A1_0 = Link_matrix(a(1),alpha(1),d(1),theta(1)) ;
A2_1 = Link_matrix(a(2),alpha(2),d(2),theta(2)) ;
A3_2 = Link_matrix(a(3),alpha(3),d(3),theta(3)) ;
A4_3 = Link_matrix(a(4),alpha(4),d(4),theta(4)) ;

A2_0=A2_1*A1_0;
A3_0=A3_2*A2_1*A1_0;
A4_0=A4_3*A3_2*A2_1*A1_0;   % Teb

%% Update POSE
% Get parameters
a1     = 0.4;
a2     = 0.25;
d1     = 36.3;
d4     = 0.05;
theta1 = str2num(get(handles.edit_theta1,'String'))*pi/180;
theta2 = str2num(get(handles.edit_theta2,'String'))*pi/180;
d3     = str2num(get(handles.edit_d3,'String'));
theta4 = str2num(get(handles.edit_theta4,'String'))*pi/180;

%Update
A1_0=eval(A1_0);
A2_0=eval(A2_0);
A3_0=eval(A3_0);
A4_0=eval(A4_0);

p0 = [0;0;0];
[p1, o1] = cal_pose(A1_0,p0);
[p2, o2] = cal_pose(A2_0,p0);
[p3, o3] = cal_pose(A3_0,p0);
[p4, o4] = cal_pose(A4_0,p0);

% if get(handles.Cor1,'Value') == 1
% 
% set(handles.xvl,'String',round(p1(1),3));
% set(handles.yvl,'String',round(p1(2),3));
% set(handles.zvl,'String',round(p1(3),3));
% set(handles.rollvl,'String',round(o1(1)*180/pi,3));
% set(handles.pitchvl,'String',round(o1(2)*180/pi,3));
% set(handles.yawvl,'String',round(o1(3)*180/pi,3));
% elseif get(handles.Cor2,'Value') == 1
% % 
% set(handles.xvl,'String',round(p2(1),3));
% set(handles.yvl,'String',round(p2(2),3));
% set(handles.zvl,'String',round(p2(3),3));
% set(handles.rollvl,'String',round(o2(1)*180/pi,3));
% set(handles.pitchvl,'String',round(o2(2)*180/pi,3));
% set(handles.yawvl,'String',round(o2(3)*180/pi,3));
% % 
% elseif get(handles.Cor3,'Value') == 1
% set(handles.xvl,'String',round(p3(1),3));
% set(handles.yvl,'String',round(p3(2),3));
% set(handles.zvl,'String',round(p3(3),3));
% set(handles.rollvl,'String',round(o3(1)*180/pi,3));
% set(handles.pitchvl,'String',round(o3(2)*180/pi,3));
% set(handles.yawvl,'String',round(o3(3)*180/pi,3));
% else
% set(handles.xvl,'String',num2str(round(1000*p4(1))/1000));
% set(handles.yvl,'String',num2str(round(1000*p4(2))/1000));
% set(handles.zvl,'String',num2str(round(1000*p4(3))/1000));
% set(handles.rollvl,'String',round(o3(1)*180/pi,3));
% set(handles.pitchvl,'String',round(o4(2)*180/pi,3));
% set(handles.yawvl,'String',round(o4(3)*180/pi,3));
% end
%% Plot
cla reset
hold on
grid on

Con =[p0,p1,p2,p3,p4];
% plot coordinate
    plot_coordinate(p0(1),p0(2),p0(3),1,1,1,0);
    plot_coordinate(p1(1),p1(2),p1(3),1,1,1,1);
    plot_coordinate(p2(1),p2(2),p2(3),1,-1,-1,2);
    plot_coordinate(p3(1),p3(2),p3(3),1,-1,-1,3);
    plot_coordinate(p4(1),p4(2),p4(3),1,-1,-1,4);
%

% define links
line1=[[p0(1) p1(1)];[p0(2) p1(2)];[p0(3) p1(3)]];
line2=[[p1(1) p2(1)];[p1(2) p2(2)];[p1(3) p2(3)]];
line3=[[p2(1) p3(1)];[p2(2) p3(2)];[p2(3) p3(3)]];
line4=[[p3(1) p4(1)];[p3(2) p4(2)];[p3(3) p4(3)]];

xlabel(handles.axes_robot,'x');
ylabel(handles.axes_robot,'y');
zlabel(handles.axes_robot,'z');
xlim(handles.axes_robot,[-1 1]);
ylim(handles.axes_robot,[-1 1]);
zlim(handles.axes_robot,[-0.55 0.1]);

%base
t = (1/16:1/8:1)'*2*pi;
x = cos(t);
y = sin(t);
z = [-0.5 -0.5 -0.5 -0.5 -0.5 -0.5 -0.5 -0.5]';
fill3(handles.axes_robot,x,y,z,'g','FaceAlpha',0.25)
base_plot=plot3(handles.axes_robot,[0 0],[ 0 0 ],[0 -0.5],'linewidth',15,'color', 'black');
base_plot.Color(4)=1;
plot3(handles.axes_robot,0,0,-0.5,'rx','linewidth',5)
%link1
line1_plot=plot3(handles.axes_robot,line1(1,:),line1(2,:),line1(3,:),'linewidth',8,'color', 'blue');
line1_plot.Color(4)=0.5;
%link2
line2_plot=plot3(handles.axes_robot,line2(1,:),line2(2,:),line2(3,:),'linewidth',7,'color', 'green');
line2_plot.Color(4)=0.5;
%link3
line3_plot=plot3(handles.axes_robot,line3(1,:),line3(2,:),line3(3,:),'linewidth',6,'color', 'cyan');
line3_plot.Color(4)=0.5;
%link4
line4_plot=plot3(handles.axes_robot,line4(1,:),line4(2,:),line4(3,:),'linewidth',5,'color', 'yellow');
line4_plot.Color(4)=0.5;
plot3(handles.axes_robot,Con(1,:),Con(2,:),Con(3,:),'rx','linewidth',5)

%picker arm
picker(p4(1),p4(2),p4(3),-str2num(get(handles.yawvl,'String'))*pi/180,0.25,0.1);

rotate3d on
view(45,45)
end