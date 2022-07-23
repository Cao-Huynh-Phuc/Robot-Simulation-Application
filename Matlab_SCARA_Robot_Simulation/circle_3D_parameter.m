function [I,R,length_arc12,angle_arc12,v1,v2,v3] = circle_3D_parameter(p1,p3,p2)
%% order on circle: p1(current position) p3(mediate position) p2(next position) 
%% circle coordination plane: v2 v3 v1 (x y z)
%%
v1 = cross(p2-p3,p1-p3);
v1 = v1/norm(v1); %Unit vector orthogonal to plane of 3 points
p12 = p2-p1;
p13 = p3-p1;
pM_12 = (p1+p2)/2;
pM_13 = (p1+p3)/2;

%% Ax + B = 0 => x = A^(-1)*(-B)
% dot(P0-p1,v1) = 0
% dot(P0-(p2+p1)/2,p2-p1) = 0
% dot(P0-(p3+p1)/2,p3-p1) = 0
A = [v1'; p12'; p13'];
B = [dot(-p1,v1); dot(-pM_12,p12); dot(-pM_13,p13)];
p0 = A\(-B);


v2 = p1-p0;
R = norm(v2);
v2 = v2/R;
v3 = cross(v1,v2);
v3 = v3/norm(v3);

%% Compute length of arc p1->p2
p31 = p1-p3;
p32 = p2-p3;
angle_31_32 = acos(dot(p31,p32)/(norm(p31)*norm(p32)));
p01 = p1-p0;
p02 = p2-p0;
angle_01_02 = acos(dot(p01,p02)/(norm(p01)*norm(p02)));
if ((angle_31_32>=0) && (angle_31_32<(pi/2)))
    angle_01_02 = 2*pi - angle_01_02;
end
angle_arc12 = angle_01_02;
length_arc12 = angle_01_02*R;
I = p0;
   

