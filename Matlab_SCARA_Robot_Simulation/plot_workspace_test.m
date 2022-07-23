figure
hold on
theta1_range = (-125:5:125)*pi/180;
theta2_range = (-145:5:145)*pi/180;
theta2_range_po = (0:5:145)*pi/180;
theta2_range_ne = (-145:5:0)*pi/180;
d3_range     = -(0:0.05:0.25);
%theta4_range = (-360:10:360)*pi/180;
x_range = [];
y_range = [];
z_range = [];

theta4 = 0;
theta2 = -145*pi/180;
for i=1:length(theta1_range)
    theta1 = theta1_range(i);
    for j=1:length(d3_range)
        d3 = d3_range(j);
        pos = cal_position_workspace(theta1, theta2, d3, theta4);
        x_range = [x_range pos(1)];
        y_range = [y_range pos(2)];
        z_range = [z_range pos(3)];
    end
end

theta2 = 145*pi/180;
for i=1:length(theta1_range)
    theta1 = theta1_range(i);
    for j=1:length(d3_range)
        d3 = d3_range(j);
        pos = cal_position_workspace(theta1, theta2, d3, theta4);
        x_range = [x_range pos(1)];
        y_range = [y_range pos(2)];
        z_range = [z_range pos(3)];
    end
end

theta1 = -125*pi/180;
for i=1:length(theta2_range_ne)
    theta2 = theta2_range_ne(i);
    for j=1:length(d3_range)
        d3 = d3_range(j);
        pos = cal_position_workspace(theta1, theta2, d3, theta4);
        x_range = [x_range pos(1)];
        y_range = [y_range pos(2)];
        z_range = [z_range pos(3)];
    end
end

theta1 = 125*pi/180;
for i=1:length(theta2_range_po)
    theta2 = theta2_range_po(i);
    for j=1:length(d3_range)
        d3 = d3_range(j);
        pos = cal_position_workspace(theta1, theta2, d3, theta4);
        x_range = [x_range pos(1)];
        y_range = [y_range pos(2)];
        z_range = [z_range pos(3)];
    end
end


theta2 = 0;
for i=1:length(theta1_range)
    theta1 = theta1_range(i);
    for j=1:length(d3_range)
        d3 = d3_range(j);
        pos = cal_position_workspace(theta1, theta2, d3, theta4);
        x_range = [x_range pos(1)];
        y_range = [y_range pos(2)];
        z_range = [z_range pos(3)];
    end
end
plot3(x_range,y_range,z_range,':b')

mark = length(x_range);
d3 = 0;
for i=1:1:length(theta1_range)
    theta1 = theta1_range(i);
    for j=1:1:length(theta2_range)
        theta2 = theta2_range(j);
        pos = cal_position_workspace(theta1, theta2, d3, theta4);
        x_range = [x_range pos(1)];
        y_range = [y_range pos(2)];
        z_range = [z_range pos(3)];
    end
end
plot3(x_range(mark:end),y_range(mark:end),z_range(mark:end),'.b','MarkerSize',2)

mark = length(x_range);
d3 = -0.25;
for i=1:1:length(theta1_range)
    theta1 = theta1_range(i);
    for j=1:1:length(theta2_range)
        theta2 = theta2_range(j);
        pos = cal_position_workspace(theta1, theta2, d3, theta4);
        x_range = [x_range pos(1)];
        y_range = [y_range pos(2)];
        z_range = [z_range pos(3)];
    end
end
plot3(x_range(mark:end),y_range(mark:end),z_range(mark:end),'.b','MarkerSize',2)
%plot3(x_range,y_range,z_range,'.r')
%% Method 1
% xlin = linspace(min(x_range), max(x_range), 1000);
% ylin = linspace(min(y_range), max(y_range), 1000);
% [X,Y] = meshgrid(xlin, ylin);
% Z = griddata(x_range(1:100:end),y_range(1:100:end),z_range(1:100:end),X,Y,'cubic');
% mesh(X,Y,Z)
%% Method 2
% X = x_range(1:10:end);
% Y = y_range(1:10:end);
% Z = diag(z_range(1:10:end));
% mesh(X,Y,Z,'FaceColor','m','FaceAlpha',1,'EdgeColor','c');
%% Method 3
% X = [];
% Y = [];
% Z = [];
% m = length(theta1_range)*length(theta2_range);
% n = length(d3_range)*length(theta4_range);
% count = 0;
% for i=1:m
%     for j=1:n
%         count = count + 1;
%         X(m,n) = x_range(count);
%         Y(m,n) = y_range(count);
%         Z(m,n) = z_range(count);
%     end
% end
% mesh(X,Y,Z,'FaceColor','b','EdgeColor','y','FaceAlpha',0.3);
%%
axis equal
grid on