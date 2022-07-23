function [qx,qy,qz,vx,vy,vz,ax,ay,az] = path_linear(xA,yA,zA,xB,yB,zB,q,v,a,q_max)


for i=1:length(q)
    qx(i) = xA + q(i)*sqrt((xA-xB)^2+(yA-yB)^2)/q_max*(xB-xA)/sqrt((xA-xB)^2+(yA-yB)^2);
    qy(i) = yA + q(i)*sqrt((xA-xB)^2+(yA-yB)^2)/q_max*(yB-yA)/sqrt((xA-xB)^2+(yA-yB)^2);
    qz(i) = zA + q(i)*(zB-zA)/q_max;
    vx(i) = v(i)*sqrt((xA-xB)^2+(yA-yB)^2)/q_max*(xB-xA)/sqrt((xA-xB)^2+(yA-yB)^2);
    vy(i) = v(i)*sqrt((xA-xB)^2+(yA-yB)^2)/q_max*(yB-yA)/sqrt((xA-xB)^2+(yA-yB)^2);
    vz(i) = v(i)*(zB-zA)/q_max;
    ax(i) = a(i)*sqrt((xA-xB)^2+(yA-yB)^2)/q_max*(xB-xA)/sqrt((xA-xB)^2+(yA-yB)^2);
    ay(i) = a(i)*sqrt((xA-xB)^2+(yA-yB)^2)/q_max*(yB-yA)/sqrt((xA-xB)^2+(yA-yB)^2);
    az(i) = a(i)*(zB-zA)/q_max;    
end
