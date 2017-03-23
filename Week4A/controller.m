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

% Thrust
Kp = [200;200;100];
Kd = [40;40;20];

cmd_acc = des_state.acc + Kd.*(des_state.vel-state.vel) + Kp.*(des_state.pos-state.pos);
F = params.mass*(params.gravity + cmd_acc(3));

% Moment
M = zeros(3,1);

Kp_ang = [100;100;100];
Kd_ang = [2;2;2];

phi_des = (1/params.gravity)*(cmd_acc(1)*sin(des_state.yaw) - cmd_acc(2)*cos(des_state.yaw));
theta_des = (1/params.gravity)*(cmd_acc(1)*cos(des_state.yaw) + cmd_acc(2)*sin(des_state.yaw));

rot_des = [phi_des; theta_des; des_state.yaw];
omega_des = [0; 0; des_state.yawdot];

M = Kp_ang.*(rot_des-state.rot) + Kd_ang.*(omega_des-state.omega);

% =================== Your code ends here ===================

end
