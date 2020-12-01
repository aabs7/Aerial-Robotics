function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

z = s(1);
z_dot = s(2);
z_des = s_des(1);
z_des_dot = s_des(2);

e = z_des - z;
e_dot = z_des_dot - z_dot;

m = params.mass;
g = params.gravity;

kp = 100;
kv = 11;
%u = 0;
u = m * ( kp * e + kv * e_dot + g);
% FILL IN YOUR CODE HERE

end

