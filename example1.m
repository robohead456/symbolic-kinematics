%% Symbolic Kinematics Example

syms t1 t2 l1 l2 d3;

% DH table [theta, d, alpha, a]
dht = [t1+(pi/2), l1, pi/2, 0;...
       t2,        l2, pi/2, 0;...
       0,         d3, 0,    0];
   
% Joint types [0=revolute, 1=prismatic]
joints = [0 0 1];

% Instantiate the kinematics
K = kinematics(dht,joints);

% Display the jacobian
K.disp_J();