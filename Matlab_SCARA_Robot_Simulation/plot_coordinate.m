function plot_coordinate(x,y,z,A,num,axis_length)
% x,y,z: toa do bat dau cua he toa do
% x_dir, y_dir, z_dir: huong cua truc toa do tuong ung
% num: so thu tu cua he truc toa do
% vidu: 
% plot_coordinate(p0(1),p0(2),p0(3),1,1,1,0);
hold on
axis_x0 = [axis_length 0 0 1]';
axis_y0 = [0 axis_length 0 1]';
axis_z0 = [0 0 axis_length 1]';
axis_x  = A*axis_x0;
axis_y  = A*axis_y0;
axis_z  = A*axis_z0;
% X = [x x x];
% Y = [y y y];
% Z = [z z z];
% U = [axis_x(1) axis_y(1) axis_z(1)];
% V = [axis_x(2) axis_y(2) axis_z(2)];
% W = [axis_x(3) axis_y(3) axis_z(3)];
% quiver3(X,Y,Z,U,V,W,0,'r','LineWidth',1.5)

% quiver3(x,y,z,axis_x(1),axis_x(2),axis_x(3),0,'r','LineWidth',1.5)
% text(axis_x(1),axis_x(2),axis_x(3),(['x',num2str(num)]))
% quiver3(x,y,z,axis_y(1),axis_y(2),axis_y(3),0,'g','LineWidth',1.5)
% text(axis_y(1),axis_y(2),axis_y(3),(['y',num2str(num)]))
% quiver3(x,y,z,axis_z(1),axis_z(2),axis_z(3),0,'b','LineWidth',1.5)
% text(axis_z(1),axis_z(2),axis_z(3),(['z',num2str(num)]))

line([x axis_x(1)],[y axis_x(2)],[z axis_x(3)],'linewidth',1.5,'color', 'red')
text(axis_x(1),axis_x(2),axis_x(3),(['x',num2str(num)]))
line([x axis_y(1)],[y axis_y(2)],[z axis_y(3)],'linewidth',1.5,'color', 'green')
text(axis_y(1),axis_y(2),axis_y(3),(['y',num2str(num)]))
line([x axis_z(1)],[y axis_z(2)],[z axis_z(3)],'linewidth',1.5,'color', 'blue')
text(axis_z(1),axis_z(2),axis_z(3),(['z',num2str(num)]))
end