function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

u1 = 0;
u2 = 0;

%states
y = state.pos(1);
z = state.pos(2);
y_des = des_state.pos(1);
z_des = des_state.pos(2);
phi = state.rot;

%velocities
omega = state.omega;
y_dot = state.vel(1);
z_dot = state.vel(2);
y_des_dot = des_state.vel(1);
z_des_dot = des_state.vel(2);

% desired acceleration
y_ddot = des_state.acc(1);
z_ddot = des_state.acc(2);

%other parameters
m = params.mass;
g = params.gravity;
Ixx = params.Ixx;

kp_z = 60;
kv_z =8;

kp_y = 20.0;
kv_y = 8.0;

kp_phi = 1000;
kv_phi = 50;

phi_c_dot = 0;
phi_c_ddot = 0;


phi_c = (-1 / g) * (y_ddot + kv_y * (y_des_dot - y_dot) + kp_y * (y_des - y));
u1 = m * (g + z_ddot + kv_z *(z_des_dot - z_dot) + kp_z * (z_des - z));
u2 = Ixx * (phi_c_ddot + kv_phi * (phi_c_dot-omega) + kp_phi * (phi_c - phi));
% FILL IN YOUR CODE HERE

end

