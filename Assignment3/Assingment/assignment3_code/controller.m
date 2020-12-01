function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
%parameters
g = params.gravity;
m = params.mass;


% Thrust
kp_f = [200.0;200.0;200.0];
kd_f = [20;20;20];
command_acc = des_state.acc + kd_f.*(des_state.vel - state.vel) + kp_f.*(des_state.pos - state.pos);
F = m * (g + command_acc(3));


% Moment
M = zeros(3,1);
kp_a = [1000.0;1000.0;1000.0];
kd_a = [1.0;1.0;1.0];
yaw_des = des_state.yaw;
r1_acc_des = command_acc(1);
r2_acc_des = command_acc(2);

phi_des = (1/g) * (r1_acc_des * sin(yaw_des) - r2_acc_des * cos(yaw_des));
theta_des = (1/g) * (r1_acc_des * cos(yaw_des) + r2_acc_des * sin(yaw_des));

rot_des = [phi_des;theta_des;yaw_des];
omega_des = [0;0;des_state.yawdot];
M = kp_a.*(rot_des - state.rot) + kd_a .* (omega_des - state.omega);
% =================== Your code ends here ===================

end
