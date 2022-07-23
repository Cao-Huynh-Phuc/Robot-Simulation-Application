function [qx,qy,qz,vx,vy,vz,ax,ay,az] = path_circular(I,R,v2,v3,q,v,a)
%%
%Circular interpolation
%%
for i=1:length(q)
    angle = q(i)/R;
    position = I + R*(v2*cos(angle) + v3*sin(angle));
    qx(i) = position(1);
    qy(i) = position(2);
    qz(i) = position(3);
    
    velocity = -v2*sin(angle)*v(i) + v3*cos(angle)*v(i);
    vx(i) = velocity(1);
    vy(i) = velocity(2);
    vz(i) = velocity(3);
    
    acceleration = -v2*(cos(angle)*v(i)*v(i)/R + sin(angle)*a(i)) + v3*(-sin(angle)*v(i)*v(i)/R + cos(angle)*a(i));
    ax(i) = acceleration(1);
    ay(i) = acceleration(2);
    az(i) = acceleration(3);
    
    
end

