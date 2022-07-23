function [q,v,a,t,v_max] = trajectory_S_curve(q_max,v_max,a_max,Ts,handles,enable_report)
global enable_noncentralize_control
if v_max > sqrt(0.5*q_max*a_max)*2/3
    v_max = sqrt(0.5*q_max*a_max)*2/3;
    if (enable_report == true)
        waitfor(warndlg(['Maximum velocity is impossible and is setted to ',num2str(v_max*1000),' mm/s'],'Warning'));      
        set(handles.edit_v_max,'String',v_max*1000);
    end
end
tc = v_max/a_max;
tf = (q_max + 2*a_max*tc^2)/(a_max*tc);
%t_m = (q_max-2*a_max*tc^2)/(a_max*tc);
t_m = tf-4*tc;

%% Dieu kien
% s_max > 2*a_max*tc^2 = 2*v_max^2/a_max
% v_max < sqrt(0.5*s_max*a_max)

%N = round(tf/Ts);

if (enable_noncentralize_control == false)
    N = round(tf/10*25);
    if (N>51)
        N = 51;
    end
    if (N<11)
        N = 11;
    end
    t = linspace(0,tf,N);
else
    t = 0:0.01:tf;
    if (t(end) ~= tf)
        t(end + 1) = t(end) + 0.01;
        t(end + 1) = t(end) + 0.01;
    end
end
Jerk = a_max/tc;
for i = 1:length(t)
    if t(i) <= tc
        a(i) = Jerk*t(i);
        v(i) = 1/2*Jerk*t(i)^2;
        q(i) = 1/6*Jerk*t(i)^3;
        %a1 = Jerk*tc;
        %v1 = 1/2*Jerk*tc^2;
        %s1 = 1/6*Jerk*tc^3;
    elseif t(i) <= 2*tc
        v1 = -a_max*tc;
        q1 = 1/3*a_max*tc^2;
        a(i) = -Jerk*t(i) + 2*a_max;
        v(i) = -1/2*Jerk*t(i)^2 + 2*a_max*t(i) + v1;
        q(i) = -1/6*Jerk*t(i)^3 + a_max*t(i)^2 + v1*t(i) + q1;
        %a2 = 0;
        %v2 = v(end);
        %s2 = q(end);
        %a3 = 0;
        %v3 = v2;
        %s3 = q_max-s2;
    elseif t(i) <= 2*tc + t_m
        q2 = -a_max*tc^2;
        a(i) = 0;
        v(i) = v_max;
        q(i) = v_max*t(i) + q2;
    elseif t(i) <= 3*tc + t_m
        v3 = v_max - 0.5*Jerk*(tf-2*tc)^2;
        q3 = 1/6*Jerk*(tf-2*tc)^3 - a_max*tc^2;
        a(i) = -Jerk*t(i) + Jerk*(tf - 2*tc);
        v(i) = -0.5*Jerk*t(i)^2 + Jerk*(tf-2*tc)*t(i) + v3;
        q(i) = -1/6*Jerk*t(i)^3 + 0.5*Jerk*(tf-2*tc)*t(i)^2 + v3*t(i) + q3;
        %a5 = 0;
        %v5 = 0;
        %s5 = q_max;
    elseif (t(i)<=tf)
        v4 = 0.5*Jerk*tf^2;
        q4 = -1/6*Jerk*tf^3 - 2*a_max*tc^2 + a_max*tc*tf;
        a(i) = Jerk*t(i) - Jerk*tf;
        v(i) = 0.5*Jerk*t(i)^2 - Jerk*tf*t(i) + v4;
        q(i) = 1/6*Jerk*t(i)^3 - 0.5*Jerk*tf*t(i)^2 + v4*t(i) + q4;
    else
        v4 = 0.5*Jerk*tf^2;
        q4 = -1/6*Jerk*tf^3 - 2*a_max*tc^2 + a_max*tc*tf;
        a(i) = Jerk*tf - Jerk*tf;
        v(i) = 0.5*Jerk*tf^2 - Jerk*tf*tf + v4;
        q(i) = 1/6*Jerk*tf^3 - 0.5*Jerk*tf*tf^2 + v4*tf + q4;        
    end
end