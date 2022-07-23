function J = jacobian_matrix(theta1, theta2, d3, theta4)
% Get parameters
global a1 a2 d1 d4
if (isempty(a1))
    a1 = 0.45;
    a2 = 0.4;
    d1 = 0;
    d4 = -0.06;
end

%syms a1 a2 d1 d3 d4 theta1 theta2 theta4
a     = [a1    ; a2     ;  0  ; 0     ];
alpha = [0     ; 0      ;  0  ; pi    ];
d     = [d1    ; 0      ;  d3 ; d4    ];
theta = [theta1; theta2 ;  0  ; theta4];

%% FK Matrix
A0_0 = eye(4);
A1_0 = Link_matrix(a(1),alpha(1),d(1),theta(1)) ;
A2_1 = Link_matrix(a(2),alpha(2),d(2),theta(2)) ;
A3_2 = Link_matrix(a(3),alpha(3),d(3),theta(3)) ;
A4_3 = Link_matrix(a(4),alpha(4),d(4),theta(4)) ;

% A2_0=A2_1*A1_0;
% A3_0=A3_2*A2_1*A1_0;
% A4_0=A4_3*A3_2*A2_1*A1_0;   % Teb
A2_0=A1_0*A2_1;
A3_0=A1_0*A2_1*A3_2;
A4_0=A1_0*A2_1*A3_2*A4_3;   % Teb

%%
z0 = [0 0 1]';
p0 = [0 0 0]';
z1 = A1_0(1:3,3);
p1 = A1_0(1:3,4);
z2 = A2_0(1:3,3);
p2 = A2_0(1:3,4);
z3 = A3_0(1:3,3);
p3 = A3_0(1:3,4);
z4 = A4_0(1:3,3);
p4 = A4_0(1:3,4);

J = [cross(z0,(p4-p0)) cross(z1,(p4-p1))     z2     cross(z3,(p4-p3));...
           z0                z1          zeros(3,1)       z3];
       