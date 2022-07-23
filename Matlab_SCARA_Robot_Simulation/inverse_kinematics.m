function IsPossible = inverse_kinematics(px,py,pz,yaw,disable_kinematics_singularity,enable_report)
global theta1 theta2 d3 theta4
if (isempty(theta1))
    theta1 = 0.0;
    theta2 = 0.0;
    d3 = 0.0;
    theta4 = 0.0;
end
max_sin_cos = 1+1e-6;
IsPossible = true;
%% Update POSE
% Get parameters
global a1 a2 d1 d4
if (isempty(a1))
    a1 = 0.45;
    a2 = 0.4;
    d1 = 0;
    d4 = -0.06;
end
IsAvailable = true;

d3_new = -((d1-(-d4)) - pz);
if ((d3_new>0)||(d3_new<-0.35))
    IsAvailable = false;
end
c2 = (px^2+py^2-a1^2-a2^2)/(2*a1*a2);
s2_po = sqrt(abs(1-c2^2));
s2_ne = -s2_po;
s2 = s2_po;
s1_1 = ((a1+a2*c2)*py-a2*s2*px)/(px^2+py^2);
c1_1 = ((a1+a2*c2)*px+a2*s2*py)/(px^2+py^2);
theta1_1 = atan2(s1_1,c1_1);
s2 = s2_ne;
s1_2 = ((a1+a2*c2)*py-a2*s2*px)/(px^2+py^2);
c1_2 = ((a1+a2*c2)*px+a2*s2*py)/(px^2+py^2);
theta1_2 = atan2(s1_2,c1_2);    

if ((theta1_1>=(-125*pi/180))&&(theta1_1<=(125*pi/180))&&(theta1_2>=(-125*pi/180))&&(theta1_2<=(125*pi/180))&&...
    (abs(s1_1)<=max_sin_cos)&&(abs(c1_1)<=max_sin_cos)&&(abs(s1_2)<=max_sin_cos)&&(abs(c1_2)<=max_sin_cos))
    if (abs(theta1 - theta1_1) < abs(theta1 - theta1_2))
        theta1_new = theta1_1;
        s2 = s2_po;
    else
        theta1_new = theta1_2;
        s2 = s2_ne;
    end
elseif ((theta1_1>=(-125*pi/180))&&(theta1_1<=(125*pi/180))&&(abs(s1_1)<=max_sin_cos)&&(abs(c1_1)<=max_sin_cos))
    theta1_new = theta1_1;
    s2 = s2_po;
elseif ((theta1_2>=(-125*pi/180))&&(theta1_2<=(125*pi/180))&&(abs(s1_2)<=max_sin_cos)&&(abs(c1_2)<=max_sin_cos))
    theta1_new = theta1_2;
    s2 = s2_ne;
else
    IsAvailable = false;
end

theta2_new = atan2(s2,c2);
if ((theta2_new<(-145*pi/180))||(theta2_new>(145*pi/180))||(abs(s2)>max_sin_cos)||(abs(c2)>max_sin_cos))
    IsAvailable = false;
end

if (IsAvailable == true)    
    theta4_new = yaw - theta1_new - theta2_new;
    while (theta4_new<(-pi))
        theta4_new = theta4_new+2*pi;
    end
    while (theta4_new>(pi))
        theta4_new = theta4_new-2*pi;
    end
%     if ((theta4_new<(-360*pi/180))||(theta4_new>(360*pi/180)))
%         IsAvailable = false;
%     end
end



if (IsAvailable == true)
    J = jacobian_matrix(theta1_new, theta2_new, d3_new, theta4_new);
    if ((abs(det(J([1 2 3 6],:)))>1e-4) || (disable_kinematics_singularity == true))
        %theta1 = get(handles.slider_theta1,'value')/180*pi;
        theta1 = theta1_new;
        %set(handles.slider_theta1,'value',theta1/pi*180);
        %theta2 = get(handles.slider_theta2,'value')/180*pi;
        theta2 = theta2_new;
        %set(handles.slider_theta2,'value',theta2/pi*180);
        %d3 = -get(handles.slider_d3,'value');
        d3 = d3_new;
        %set(handles.slider_d3,'value',-d3);
        %theta4 = get(handles.slider_theta4,'value')/180*pi;
        theta4 = theta4_new;
        %set(handles.slider_theta4,'value',theta4/pi*180);
    else
        if (enable_report == true)       
            waitfor(warndlg('Kinematics Singularity!','Warning'));
        end
        IsPossible = false;
    end
else
    if (enable_report == true)
        waitfor(warndlg('Workspace Singularity!','Warning'));  
    end
    IsPossible = false;
end

