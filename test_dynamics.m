clear all; 
clc;

%% Load model
mdl_puma560;
robot = p560;

%% Create controller
controller = QpController(robot);

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
[t, y] = ode23tb(@(t,y) dynamics(t,y,robot, controller, use_friction), tspan, y0);
q_traj = y(:,1:robot.n);

%% Plot results
disp('plotting')
tcp_traj = robot.fkine(q_traj);
end_point = tcp_traj(:,:,end)
robot.plot(q_traj)
