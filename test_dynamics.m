clear all; 
clc;

%% Load model
mdl_puma560;
global robot;
robot = p560;

% qn = (robot.qlim(:,2)' + robot.qlim(:,1)')/2.0;

qn = [ 0    0.5000    3.1400         0   -0.5000         0];
robot.qlim(:,1) = qn' - pi; 
robot.qlim(:,2) = qn' + pi;
% initial state
y0 = [qn'; zeros(robot.n,1)];
torque_limit = 40;
%% Create controller
using_constraints = true;
global controller;
controller = QpController(using_constraints, torque_limit);
global torques;
torques = [];
global torque_times;
torque_times = [];
%% Simulate execution
% time scale
step = 0.01;
tend = 4;
tspan = [0.0 : step : tend];


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

%% Sample timeseries
q_ts_raw = timeseries(q_traj, t);
tau_ts_raw = timeseries(torques, torque_times);

q_ts_sampled = resample(q_ts_raw, tspan);
q_traj = q_ts_sampled.data;
t_traj = q_ts_sampled.time;

tau_ts_sampled = resample(tau_ts_raw, tspan);
tau_traj = tau_ts_sampled.data;

tcp_traj = robot.fkine(q_traj);
end_point = tcp_traj(:,:,end)
end_posture = q_traj(end,:)

%% Plot results
disp('plotting')
robot.plot(q_traj, 'delay', step)

%%
figure()
for i = 1:robot.n
    subplot(2,3,i)
    plot(t_traj, q_traj(:,i), 'b')
    ylim([robot.qlim(i,1), robot.qlim(i,2)])
    ylabel('joint position (rad)')
    xlabel('t (sec)')
    title(sprintf('Joint %i',i))
end

figure()
for i = 1:robot.n
    subplot(2,3,i)
    plot(t_traj, tau_traj(:,i), 'b')
    ylim([-torque_limit; torque_limit])
    ylabel('tau (Nm)')
    xlabel('t (sec)')
    title(sprintf('Joint %i',i))
end
