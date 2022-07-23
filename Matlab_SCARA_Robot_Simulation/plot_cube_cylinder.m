function plot_cube_cylinder(x0,y0,z0,r,h,colr,value_3dRobotView_opacity)
% x0 y0 z0: toa do tam
% r: ban kinh day
% h: chieu cao
[X,Y,Z] = cylinder(r,100);
X = X + x0;
Y = Y + y0;
Z = Z*h + z0;
surf(X,Y,Z,'facecolor',colr,'LineStyle','none','FaceAlpha',value_3dRobotView_opacity);
%hold on
fill3(X(1,:),Y(1,:),Z(1,:),colr,'FaceAlpha',value_3dRobotView_opacity)
fill3(X(2,:),Y(2,:),Z(2,:),colr,'FaceAlpha',value_3dRobotView_opacity)
%axis equal