
syms q1 q2 q3 d1 d2 d3 a1 a2 a3 alpha1 alpha2 alpha3 L1 L2 L3 zasuk1 premik2 premik3 L2P2 L3P3; % Define symbolic variables
% Define symbolic variables
q1 = zasuk1;
q2 = 0; 
q3 = 0;
d1 = L1;
d2 = L2P2; %d2=L2+premik2
d3 = L3P3; %d3=L3+premik3
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
A01 = dh_transform(dh_params(1,:));
A12 = dh_transform(dh_params(2,:));
A23 = dh_transform(dh_params(3,:));
T03 = A01*A12*A23;
T13 = A12*A23;
T23 = A23;

% Define the position of the end effector
p = T03(1:3,4);

% Calculate the Jacobian matrix
J1 = jacobian(p, [q1, d2, d3])
J2 = jakobizracun(T03, T13, T23)
% Calculate the geometric Jacobian matrix
Jg = simplify(inv(T03(1:3,1:3))*J1)

function J = jakobizracun(T03, T13, T23)
% jakobizracun(T) izracuna analiticno jakobijevo matriko ta dano
% transformacijo vrha
n03 = T03(1:end-1,1);
o03 = T03(1:end-1,2);
a03 = T03(1:end-1,3);
p03 = T03(1:end-1,4);

n13 = T13(1:end-1,1);
o13 = T13(1:end-1,2);
a13 = T13(1:end-1,3);
p13 = T13(1:end-1,4);

n23 = T23(1:end-1,1);
o23 = T23(1:end-1,2);
a23 = T23(1:end-1,3);
p23 = T23(1:end-1,4);

%rotacija
st1 = stolpecJmatrikeROTACIJA(n03,o03,a03,p03);
%tranzlacija
st2 = stolpecJmatrikeTRANSLACIJA(n13,o13,a13,p13);
%tranzlacija
st3 = stolpecJmatrikeTRANSLACIJA(n23,o23,a23,p23);

J=[st1, st2, st3];
end

function st = stolpecJmatrikeROTACIJA(n,o,a,p)
st = [-n(1)*p(2)+n(2)*p(1);
       -o(1)*p(2)+o(2)*p(1);
       -a(1)*p(2)+a(2)*p(1);
       n(3);
       o(3);
       a(3)];
end

function st = stolpecJmatrikeTRANSLACIJA(n,o,a,p)
st = [n(3);
      o(3);
      a(3);
      0;
      0;
      0];
end

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