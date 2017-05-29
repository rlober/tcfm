close all;
clear all; 
clc;

%% Load model
mdl_puma560;
global robot;
robot = p560;

% qn = (robot.qlim(:,2)' + robot.qlim(:,1)')/2.0;

global qn;
qn = [ 0    0.5000    3.1400         0   -0.5000         0];

robot.qlim(:,1) = qn' - pi; 
robot.qlim(:,2) = qn' + pi;
% initial state
y0 = [qn'; zeros(robot.n,1)];
torque_limit = 40;
%% Create controller
using_constraints = true;
compute_metrics = false;

% eePosRef = [-0.25; 0.10; -0.15];
% elPosRef = [-0.32; 0.23;  0.25];
eePosRef = [0.25; 0.10; -0.22];
elPosRef = [0.32; 0.23;  0.2];


eePoseTask = EEPoseTask(robot, 1.0, 10.0, 0.2);
poseRef = [1 0 0 0.4; 0 1 0 -0.5; 0 0 1 -0.4; 0 0 0 1];
% eePoseTask.setDesired(poseRef, [], []);

eePositionTask = EETask(robot, 1.0, 10.0, 0.2);
posRef = [-0.25; 0.25; 0.25];
eePositionTask.setDesired(eePosRef);


elbowPositionTask = ElbowPositionTask(robot, 1.0, 10.0, 0.2);
% posRef = [0.0; 0.25; 0.25];
elbowPositionTask.setDesired(elPosRef);

jointPosTask = PostureTask(robot, 1.0, 10.0, 0.2);

% tasks = {eePositionTask, jointPosTask};
tasks = {eePositionTask, elbowPositionTask};

torqueConstraint = TorqueConstraint(robot, -torque_limit, torque_limit);
positionConstraint = JointPositionConstraint(robot, robot.qlim(:,1), robot.qlim(:,2));

constraints = {torqueConstraint, positionConstraint};

global controller;
controller = QpController(tasks, constraints, using_constraints, compute_metrics, torque_limit);
global torques;
torques = [];
global torque_times;
torque_times = [];
%% Simulate execution
% time scale
step = 0.01;
tend = 2;%4;
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
clf;
sphere_radius = 0.05;
plot_sphere(eePosRef, sphere_radius, 'blue');
elPos = elbowPositionTask.sub_robot.fkine(qn(1:3));
elPos = elPos(1:3,4);
plot_sphere(elPosRef, sphere_radius, 'red');

robot.plot(q_traj, 'delay', step);

%%
fig = figure();
fig.Name = 'Joint Position Curves';
for i = 1:robot.n
    subplot(2,3,i)
    plot(t_traj, q_traj(:,i), 'b')
    ylim([robot.qlim(i,1), robot.qlim(i,2)])
    ylabel('joint position (rad)')
    xlabel('t (sec)')
    title(sprintf('Joint %i',i))
end

fig = figure();
fig.Name = 'Torque Curves';
for i = 1:robot.n
    subplot(2,3,i)
    plot(t_traj, tau_traj(:,i), 'b')
    ylim([-torque_limit; torque_limit])
    ylabel('tau (Nm)')
    xlabel('t (sec)')
    title(sprintf('Joint %i',i))
end

%% Parse metric data
if compute_metrics
    n_pts = size(controller.metric_data,1);
    times = zeros(n_pts,1);
    c_sum_dist = zeros(n_pts,1);
    c_sum_cent_dist = zeros(n_pts,1);
    c_sum_costs = zeros(n_pts,1);
    f_sum_dist = zeros(n_pts,1);
    f_cen_to_cen_dist = zeros(n_pts,1);
    obj_el_volumes = zeros(n_pts,1);
    con_el_volumes = zeros(n_pts,1);
    
    n_max_objectives = 0;
    for i = 1:n_pts
        n_max_objectives = max([n_max_objectives, controller.metric_data{i,6}]);
    end
    
    c_costs = zeros(n_pts, n_max_objectives);

    for i = 1:n_pts
        times(i,1) = controller.metric_data{i,1};
        cm = controller.metric_data{i,2};
        c_sum_dist(i,1) = cm.sum_distance;
        c_sum_cent_dist(i,1) = cm.sum_center_distance;
        c_sum_costs(i,1) = cm.sum_of_costs;
        n_objectives = controller.metric_data{i,6};
        c_costs(i,1:n_objectives) = cm.costs_at_x_star;
       
        fm = controller.metric_data{i,3};
        f_sum_dist(i,1) = fm.sum_center_distance;
        f_cen_to_cen_dist(i,1) = fm.center_to_center_distance;
        
        obj_ellipsoid = controller.metric_data{i,4};
        obj_el_volumes(i,1) = obj_ellipsoid.volume;
        
        con_ellipsoid = controller.metric_data{i,5};
        con_el_volumes(i,1) = con_ellipsoid.volume;

    end 
end

%% Plot metric data
if compute_metrics
    fig = figure();
    fig.Name = 'Objective Compatibility Metrics';
    subplot(2,1,1)
    plot(times,c_sum_dist)
    ylabel('c_sum_dist')
    xlabel('t (sec)')
    subplot(2,1,2)
    plot(times,c_sum_cent_dist)
    ylabel('c_sum_cent_dist')
    xlabel('t (sec)')
    
    
    fig = figure();
    fig.Name = 'Objective Cost Metrics';
    
    for i = 1:n_max_objectives
        subplot(n_max_objectives+1,1,i)
        plot(times,c_costs(:,i))
        ylabel(sprintf('cost f_%i',i))
        xlabel('t (sec)')
    end
    subplot(n_max_objectives+1,1,3)
    plot(times,c_sum_costs)
    ylabel('c_sum_costs')
    xlabel('t (sec)')
    
    fig = figure();
    fig.Name = 'Objective Feasibility Metrics';
    subplot(2,1,1)
    plot(times,f_sum_dist)
    ylabel('f_sum_dist')
    xlabel('t (sec)')
    
    subplot(2,1,2)
    plot(times,f_cen_to_cen_dist)
    ylabel('f_cen_to_cen_dist')
    xlabel('t (sec)')
    
    fig = figure();
    fig.Name = 'Ellipsoid Volumes';
    subplot(2,1,1)
    plot(times,obj_el_volumes)
    ylabel('obj_el_volumes')
    xlabel('t (sec)')
    
    subplot(2,1,2)
    plot(times,con_el_volumes)
    ylabel('con_el_volumes')
    xlabel('t (sec)')
end
