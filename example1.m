%% Symbolic Kinematics Example

syms t1 t2 d3 t4 t5 t6;
syms d2 d6;

pi = sym(pi);

% DH table [theta, d, alpha, a]
dht = [t1,  0, -pi/2, 0;...
       t2, d2,  pi/2, 0;...
       0,  d3,     0, 0;...
       t4,  0, -pi/2, 0;...
       t5,  0,  pi/2, 0;...
       t6, d6,     0, 0];
   
% Joint types [0=revolute, 1=prismatic]
joints = [0 0 1 0 0 0];

% Instantiate the kinematics
K = kinematics(dht,joints);

% Check that all transforms are valid
K.check_transforms();

% Display the jacobian
K.disp_J();