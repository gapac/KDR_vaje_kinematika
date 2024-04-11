syms q1 q2 q3 d1 d2 d3 a1 a2 a3 alpha1 alpha2 alpha3 L1 L2 L3 zasuk1 premik2 premik3 L2P2 L3P3; % Define symbolic variables
% Define symbolic variables
%q1 = zasuk1;
q1 = 0;%zato da ni v singularnosti
q2 = 0; 
q3 = 0;
%d1 = L1;
d1 = 0.1;
%d2 = L2P2; %d2=L2+premik2
d2 = 0.4;
%d3 = L3P3; %d3=L3+premik3
d3 = 0.2; 
a1 = 0;
a2 = 0;
a3 = 0;
alpha1 = 0;
alpha2 = pi/2; 
alpha3 = 0; 

% Define the DH parameters
dh_params = [q1 d1 a1 alpha1;
             q2 d2 a2 alpha2;
             q3 d3 a3 alpha3];

% Define the forward kinematics transformation matrices
A01 = dh_transform(dh_params(1,:))
A12 = dh_transform(dh_params(2,:))
A23 = dh_transform(dh_params(3,:))
T03 = A01*A12*A23


r03 = [0 0.2 0.5]
Sr03 = [0         -r03(3)   r03(2);
        r03(3)    0         -r03(1);
        -r03(2)   r03(1)    0]

R03 = T03(1:3,1:3) 
Fvrh = transpose([0 0 10 0 0 0])
TransF =[R03       zeros(3,3);
         Sr03*R03  R03]
Fbaz = TransF *  Fvrh




function T = dh_transform(dh_params)
% DH_TRANSFORM Calculates the DH transformation matrix for a given set of DH parameters
%   T = DH_TRANSFORM(DH_PARAMS) calculates the DH transformation matrix for a given set of DH parameters.
%   DH_PARAMS is a 1x4 vector containing the DH parameters [d, theta, a, alpha].
%   T is a 4x4 homogeneous transformation matrix.


theta = dh_params(1);
d = dh_params(2);
a = dh_params(3);
alpha = dh_params(4);

T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
     sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
     0 sin(alpha) cos(alpha) d;
     0 0 0 1];
end