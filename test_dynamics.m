close all;
clear all; 
clc;

%% Load model
mdl_puma560;
global robot;
robot = p560;
robot_fig = figure();
robot_fig.Name = 'Robot Animation';

global qn;
qn(1) =  3*pi/4;
qn(3) = -pi;

% initial state
y0 = [qn'; zeros(robot.n,1)];
torque_limit = 40;
%% Create controller
using_constraints = true;
compute_metrics = true;

% eePosRef = [-0.25; 0.10; -0.15];
% elPosRef = [-0.32; 0.23;  0.25];
eePosRef = [-0.25; -0.5; -0.22];
elPosRef = [-0.32; -0.23;  0.2];


% eePoseTask = EEPoseTask(robot, 1.0, 10.0, 0.2);
% poseRef = [1 0 0 0.4; 0 1 0 -0.5; 0 0 1 -0.4; 0 0 0 1];

eePositionTask = EETask(robot, 1.0, 10.0, 0.2);
eePositionTask.setDesired(eePosRef);


elbowPositionTask = ElbowPositionTask(robot, 1.0, 10.0, 0.2);
% elbowPositionTask.setDesired(elPosRef);

jointPosTask = PostureTask(robot, 0.0001, 10.0, 0.2);

tasks = {eePositionTask};
% tasks = {eePositionTask, jointPosTask};
% tasks = {eePositionTask, elbowPositionTask, jointPosTask};

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
tend = 8;
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
end_point = tcp_traj(:,:,end);
end_posture = q_traj(end,:);

%% Get Task Reference Data

n_tasks = size(controller.tasks,2);
task_ref_data = {};
for i = 1:n_tasks
   task_refs = controller.tasks{i}.references;
   n_dof = size(task_refs{1,2},1);
   n_steps = size(task_refs,1);
   tmp_times = zeros(n_steps,1);
   tmp_pos = zeros(n_steps,n_dof);
   tmp_vel = zeros(n_steps,n_dof);
   tmp_acc = zeros(n_steps,n_dof);
   for j = 1:n_steps
      tmp_times(j,1) = task_refs{j,1};
      tmp_pos(j,:) = task_refs{j,2}'; 
      tmp_vel(j,:) = task_refs{j,3}'; 
      tmp_acc(j,:) = task_refs{j,4}'; 
   end
   task_ref_data = [task_ref_data; {tmp_times, tmp_pos, tmp_vel, tmp_acc}];
end

%% Plot results
disp('Displaying Animation')
clf(robot_fig);
sphere_radius = 0.05;
plot_sphere(eePosRef, sphere_radius, 'blue');
plot_sphere(elPosRef, sphere_radius, 'red');

% elPos = elbowPositionTask.sub_robot.fkine(qn(1:3));
% elPos = elPos(1:3,4);
hold on;
for i = 1:n_tasks
    pos= task_ref_data{i,2};
    Xs = pos(:,1);
    Ys = pos(:,2);
    Zs = pos(:,3);
    plot3(Xs,Ys,Zs);
end
hold off;

robot.plot(q_traj, 'delay', step);

%% Plot Joint Positions and Torques
fig1 = figure();
fig1.Name = 'Joint Position Curves';
rad_to_deg = (180/pi);
q_traj_deg = rad_to_deg*q_traj;
qlims_deg = rad_to_deg*robot.qlim;
for i = 1:robot.n
    subplot(2,3,i)
    plot(t_traj, q_traj_deg(:,i), 'b')
    ylim([qlims_deg(i,1), qlims_deg(i,2)])
    ylabel('joint position (degree)')
    xlabel('t (sec)')
    title(sprintf('Joint %i',i))
end

fig2 = figure();
fig2.Name = 'Torque Curves';
for i = 1:robot.n
    subplot(2,3,i)
    plot(t_traj, tau_traj(:,i), 'b')
    ylim([-torque_limit; torque_limit])
    ylabel('tau (Nm)')
    xlabel('t (sec)')
    title(sprintf('Joint %i',i))
end

%% Plot task references
for i = 1:n_tasks
  
   tmp_times = task_ref_data{i,1};
   tmp_pos = task_ref_data{i,2};
   tmp_vel = task_ref_data{i,3};
   tmp_acc = task_ref_data{i,4};
   n_dof = size(tmp_pos,2);
  
   fig3 = figure();
   fig3.Name = sprintf('Task %i reference trajectory',i);
   for j = 1:n_dof
       
       subplot(3, n_dof, j);
       plot(tmp_times, tmp_pos(:,j))
       xlabel('t (sec)')
       ylabel('pos (m)')
       
       subplot(3, n_dof, j+n_dof);
       plot(tmp_times, tmp_vel(:,j))
       xlabel('t (sec)')
       ylabel('vel (m/s)')
       
       subplot(3, n_dof, j+2*n_dof);
       plot(tmp_times, tmp_acc(:,j))
       xlabel('t (sec)')
       ylabel('acc (m/s^2)')
       
   end
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
    
    n_constraints = size(controller.metric_data{1,7},1);
    GX = zeros(n_pts, n_constraints);
    H = zeros(n_pts, n_constraints);

    
    obj_el_volumes = zeros(n_pts,1);
    con_el_volumes = zeros(n_pts,1);
    
    n_max_objectives = 0;
    for i = 1:n_pts
        n_max_objectives = max([n_max_objectives, controller.metric_data{i,6}]);
    end
    
    c_costs = zeros(n_pts, n_max_objectives);
    
    f_ellipsoid_inequality_measure = zeros(n_pts,n_max_objectives);
    f_optimum_is_in_ellipsoid = zeros(n_pts,n_max_objectives);
    f_x_star_in_con_ellipsoid = zeros(n_pts,1);

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
        f_ellipsoid_inequality_measure(i,1:n_objectives) = fm.ellipsoid_inequality_measure;
        f_optimum_is_in_ellipsoid(i,1:n_objectives) = fm.optimum_is_in_ellipsoid; 
        f_x_star_in_con_ellipsoid(i,1) = fm.x_star_in_con_ellipsoid;
        
        obj_ellipsoid = controller.metric_data{i,4};
        obj_el_volumes(i,1) = obj_ellipsoid.volume;
        
        con_ellipsoid = controller.metric_data{i,5};
        con_el_volumes(i,1) = con_ellipsoid.volume;
        
        
        GX(i,:) = (controller.metric_data{i,7}*controller.metric_data{i,9})';
        H(i,:) = controller.metric_data{i,8}';
    end 
end

%% Plot metric data
if compute_metrics
    fig4 = figure();
    fig4.Name = 'Objective Compatibility Metrics';
    subplot(2,1,1)
    plot(times,c_sum_dist)
    ylabel('c sum dist')
    xlabel('t (sec)')
    subplot(2,1,2)
    plot(times,c_sum_cent_dist)
    ylabel('c sum cent dist')
    xlabel('t (sec)')
    
    
    fig4 = figure();
    fig4.Name = 'Objective Cost Metrics';
    
    for i = 1:n_max_objectives
        subplot(n_max_objectives+1,1,i)
        plot(times,c_costs(:,i))
        ylabel(sprintf('cost f_%i',i))
        xlabel('t (sec)')
    end
    subplot(n_max_objectives+1,1,n_max_objectives+1)
    plot(times,c_sum_costs)
    ylabel('c sum costs')
    xlabel('t (sec)')
    
    fig5 = figure();
    fig5.Name = 'Objective Feasibility Metrics';
    subplot(2,1,1)
    plot(times,f_sum_dist)
    ylabel('f sum dist')
    xlabel('t (sec)')
    
    subplot(2,1,2)
    plot(times,f_cen_to_cen_dist)
    ylabel('f cen to cen dist')
    xlabel('t (sec)')
    
    fig6 = figure();
    fig6.Name = 'Ellipsoid Volumes';
    subplot(2,1,1)
    plot(times,obj_el_volumes)
    ylabel('obj el volumes')
    xlabel('t (sec)')
    
    subplot(2,1,2)
    plot(times,con_el_volumes)
    ylabel('con el volumes')
    xlabel('t (sec)')
    
    fig7 = figure();
    fig7.Name = 'Ellipsoid Equation Evaluation';
    for i = 1:n_max_objectives
        subplot(n_max_objectives+1,1,i)
        plot(times,f_ellipsoid_inequality_measure(:,i))
        ylabel(sprintf('ellipsoid inequality measure f_%i',i))
        xlabel('t (sec)')
    end
    subplot(n_max_objectives+1,1,n_max_objectives+1)
    plot(times,f_x_star_in_con_ellipsoid)
    ylabel('f x^* in con ellipsoid')
    xlabel('t (sec)')
    
    
    n_con_types = size(controller.constraints,2);
    n_con_eqns = robot.n * 2;
    k = 1;
    for j = 1:n_con_types
       
       fig8 = figure();
       fig8.Name = 'Constraint Evaluation at x^*';
       m = 1;
       for i = k:n_con_eqns*j
           subplot(2, (n_con_eqns/2), m)
           plot(times, H(:,i), 'Color', [0.6, 0.6, 0], 'LineWidth', 3);
           hold on;
           plot(times, GX(:,i), 'Color', [0.0, 0.6, 0], 'LineWidth', 3);
           for f = 1:size(H,1)
              if ( GX(f,i) > H(f,i) )
                 plot(times(f,1), GX(f,i), 'LineStyle', 'none', 'Marker', 'o', 'MarkerFaceColor', [0.6, 0, 0], 'MarkerSize', 4)  
              end
           end
           
           hold off;
           m = m+1;
       end
       k = k+n_con_eqns;
    end
    
        
    
end
