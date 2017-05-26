clear all; 
clc;

%% Load model
mdl_puma560;
global robot;
robot = p560;

%% Create controller
using_constraints = true;
global controller;
controller = QpController(using_constraints);

%% Simulate execution
% time scale
step = 0.01;
tend = 2.0;
tspan = [0.0 : step : tend];

% initial state
y0 = [qn'; zeros(robot.n,1)];

% friction
use_friction = false; % bug slows down integ

% foward dynamics integration
disp('simulating')
global stop_integration;
stop_integration = false;
opts=odeset('Events',@odeStop);
[t, y] = ode45(@(t,y) dynamics(t,y, use_friction), tspan, y0);
size(t)
% [t, y] = simpleDynamicsIntegration( use_friction, tspan, y0 );
q_traj = y(:,1:robot.n);

%% Plot results
disp('plotting')
tcp_traj = robot.fkine(q_traj);
end_point = tcp_traj(:,:,end)
end_posture = q_traj(end,:)
robot.plot(q_traj)
