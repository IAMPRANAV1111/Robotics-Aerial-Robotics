function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
  % state.pos = [y; z]; 
 %  state.vel = [y_dot; z_dot]; 
 %  state.rot = [phi];
 %  state.omega = [phi_dot];
%
%   des_state: The desired states are:
%   des_state.pos = [y; z];
 %  des_state.vel = [y_dot; z_dot];
 %  des_state.acc = [y_ddot; z_ddot];
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

y = state.pos(1);
y_dot = state.vel(1);
y_T = des_state.pos(1);
y_T_dot = des_state.vel(1);
y_T_dot_dot = des_state.acc(1);

z = state.pos(2);
z_dot = state.vel(2);
z_T = des_state.pos(2);
z_T_dot = des_state.vel(2);
z_T_dot_dot = des_state.acc(2);

phi = state.rot;
phi_dot = state.omega;
phi_T_dot = 0;
phi_T_dot_dot = 0;

kp_z = 80;
kv_z = 20;

kp_y = 20;
kv_y = 5;

kp_phi = 1000;
kv_phi = 10;

%u1 = 0;
%u2 = 0;

% FILL IN YOUR CODE HERE
z_c_dot_dot = (z_T_dot_dot + (kv_z * (z_T_dot - z_dot)) + (kp_z * (z_T - z)));
u1 = (params.mass)*(params.gravity) + (params.mass)*(z_c_dot_dot);


y_c_dot_dot = (y_T_dot_dot + (kv_y * (y_T_dot - y_dot)) + (kp_y * (y_T - y)));
phi_c = (-1.0)*(y_c_dot_dot / params.gravity);
   

phi_c_dot_dot = (phi_T_dot_dot + (kv_phi * (phi_T_dot - phi_dot)) + (kp_phi * (phi_c - phi)));
u2 = params.Ixx * (phi_c_dot_dot);

% FILL IN YOUR CODE HERE

end

