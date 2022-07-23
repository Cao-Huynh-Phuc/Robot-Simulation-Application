function [I,R,pC,q_max,angle_max,v1,v2,v3,IsPossible]= path_circular_parameter(xA,yA,zA,xB,yB,zB,v_max,a_max,mode_trajectory,Ts,disable_kinematics_singularity,handles)
%%
%Circular interpolation
%%
global a1 a2 d1 d4
if (isempty(a1))
    a1 = 0.45;
    a2 = 0.4;
    d1 = 0;
    d4 = -0.06;
end
global theta1 theta2 d3 theta4
if (isempty(theta1))
    theta1 = 0.0;
    theta2 = 0.0;
    d3 = 0.0;
    theta4 = 0.0;
end
theta1_current = theta1;
theta2_current = theta2;
d3_current = d3;
theta4_current = theta4;
%% 
syms xC yC
pA = [xA yA zA]';
pB = [xB yB zB]';
pM = (pA+pB)/2;
xM = pM(1);
yM = pM(2);
zC = pM(3);
% Vector chi phuong
AB = pB - pA;
 
distace_MC = 0;
finish = false;
while (finish == false)        
    distace_MC = distace_MC + 0.1;
    IsPossible = true;
    OC = [];
    sC = solve([AB(1)*(xC-xM) + AB(2)*(yC-yM), (xC-xM)^2 + (yC-yM)^2 - distace_MC*distace_MC],[xC yC]);
    for i=1:length(sC.xC)      
        OC(i) = sqrt(eval(sC.xC(i))^2 + eval(sC.yC(i))^2);
        IsPossible = inverse_kinematics(eval(sC.xC(i)),eval(sC.yC(i)),zC,0,disable_kinematics_singularity,false);
        if (IsPossible == true)               
            theta1 = theta1_current;
            theta2 = theta2_current;
            d3 = d3_current;
            theta4 = theta4_current;
            pC = [eval(sC.xC(i)) eval(sC.yC(i)) zC]';
            [I,R,q_max,angle_max,v1,v2,v3] = circle_3D_parameter(pA,pC,pB);
            %v_max = sqrt(q_max*a_max);
            if (mode_trajectory == 0)
                [q,v,a,t,~] = trajectory_LSPB(q_max,v_max,a_max,Ts,handles,false);  
            else       
                [q,v,a,t,~] = trajectory_S_curve(q_max,v_max,a_max,Ts,handles,false);
            end
            [qx,qy,qz,~,~,~,~,~,~] = path_circular(I,R,v2,v3,q,v,a);
            for j=1:length(t)       
                IsPossible = inverse_kinematics(qx(j),qy(j),qz(j),0,disable_kinematics_singularity,false);    
                theta1_sequence(j) = theta1;
                theta2_sequence(j) = theta2;    
                if (j>2)
                    if ((abs(theta1_sequence(j)-theta1_sequence(j-1))>pi/2) || (abs(theta2_sequence(j)-theta2_sequence(j-1))>pi/2))
                        IsPossible = false;
                    end
                end
                if(IsPossible == false)
                    break
                end 
            end               
        end
        if (IsPossible == true)
            break
        end
    end
    if (IsPossible == true) 
        finish = true;
    else        
        finish = true;
        for i=1:length(OC)
            if (OC(i) < (a1+a2)) 
                finish = false;
                break
            end
        end
    end
end   
%%
% if (IsLinearPathPossible == true)
%     IsPossible = true;
%     sC = solve([AB(1)*(xC-xM) + AB(2)*(yC-yM), (xC-xM)^2 + (yC-yM)^2 - 0.01],[xC yC]);
%     for i=1:length(sC.xC)       
%         if (inverse_kinematics(eval(sC.xC(i)),eval(sC.yC(i)),zC,0, false) == true)
%             pC = [eval(sC.xC(i)) eval(sC.yC(i)) zC]';
%             break
%         end
%     end
% else    
%     OC = 0;   
%     distace_MC = 0;
%     finish = false;
%     while (finish == false)        
%         distace_MC = distace_MC + 0.1;
%         IsPossible = true;
%         OC = [];
%         sC = solve([AB(1)*(xC-xM) + AB(2)*(yC-yM), (xC-xM)^2 + (yC-yM)^2 - distace_MC*distace_MC],[xC yC]);
%         for i=1:length(sC.xC)      
%             OC(i) = sqrt(eval(sC.xC(i))^2 + eval(sC.yC(i))^2);
%             IsPossible = inverse_kinematics(eval(sC.xC(i)),eval(sC.yC(i)),zC,0, false);
%             if (IsPossible == true)               
%                 theta1 = theta1_current;
%                 theta2 = theta2_current;
%                 d3 = d3_current;
%                 theta4 = theta4_current;
%                 pC = [eval(sC.xC(i)) eval(sC.yC(i)) zC]';
%                 [I,R,q_max,angle_max,v1,v2,v3] = circle_3D_parameter(pA,pC,pB);
%                 %v_max = sqrt(q_max*a_max);
%                 if (mode_trajectory == 0)
%                     [q,v,a,t,~] = trajectory_LSPB(q_max,v_max,a_max,true);  
%                 else       
%                     [q,v,a,t,~] = trajectory_S_curve(q_max,v_max,a_max,true);
%                 end
%                 [qx,qy,qz,vx,vy,vz,ax,ay,az] = path_circular(I,R,v2,v3,q,v,a);
%                 for j=1:length(t)       
%                     IsPossible = inverse_kinematics(qx(j),qy(j),qz(j),0,false);    
%                     theta1_sequence(j) = theta1;
%                     theta2_sequence(j) = theta2;    
%                     if (j>2)
%                         if ((abs(theta1_sequence(j)-theta1_sequence(j-1))>pi/2) || (abs(theta2_sequence(j)-theta2_sequence(j-1))>pi/2))
%                             IsPossible = false;
%                         end
%                     end
%                     if(IsPossible == false)
%                         break
%                     end 
%                 end               
%             end
%             if (IsPossible == true)
%                 break
%             end
%         end
%         if (IsPossible == true) 
%             finish = true;
%         else        
%             finish = true;
%             for i=1:length(OC)
%                 if (OC(i) < (a1+a2)) 
%                     finish = false;
%                     break
%                 end
%             end
%         end
%     end   
% end

%%
if ((IsPossible == false) && (disable_kinematics_singularity == true))
%     theta1 = theta1_current;
%     theta2 = theta2_current;
%     d3 = d3_current;
%     theta4 = theta4_current;
    sC = solve([AB(1)*(xC-xM) + AB(2)*(yC-yM), xC^2 + yC^2 - 0.85^2],[xC yC]);
    for i=1:length(sC.xC)      
        %OC(i) = sqrt(eval(sC.xC(i))^2 + eval(sC.yC(i))^2);
        theta1 = theta1_current;
        theta2 = theta2_current;
        d3 = d3_current;
        theta4 = theta4_current;
        IsPossible = inverse_kinematics(eval(sC.xC(i)),eval(sC.yC(i)),zC,0,disable_kinematics_singularity,false);
        if (IsPossible == true)               
            theta1 = theta1_current;
            theta2 = theta2_current;
            d3 = d3_current;
            theta4 = theta4_current;
            pC = [eval(sC.xC(i)) eval(sC.yC(i)) zC]';
            [I,R,q_max,angle_max,v1,v2,v3] = circle_3D_parameter(pA,pC,pB);
            %v_max = sqrt(q_max*a_max);
            if (mode_trajectory == 0)
                [q,v,a,t,~] = trajectory_LSPB(q_max,v_max,a_max,Ts,handles,false);  
            else       
                [q,v,a,t,~] = trajectory_S_curve(q_max,v_max,a_max,Ts,handles,false);
            end
            [qx,qy,qz,~,~,~,~,~,~] = path_circular(I,R,v2,v3,q,v,a);
            for j=1:length(t)       
                IsPossible = inverse_kinematics(qx(j),qy(j),qz(j),0,disable_kinematics_singularity,false);    
                theta1_sequence(j) = theta1;
                theta2_sequence(j) = theta2;    
                if (j>2)
                    if ((abs(theta1_sequence(j)-theta1_sequence(j-1))>pi/2) || (abs(theta2_sequence(j)-theta2_sequence(j-1))>pi/2))
                        IsPossible = false;
                    end
                end
                if(IsPossible == false)
                    break
                end 
            end               
        end
        if (IsPossible == true)
            break
        end
    end
end
%%
[I,R,q_max,angle_max,v1,v2,v3] = circle_3D_parameter(pA,pC,pB);



