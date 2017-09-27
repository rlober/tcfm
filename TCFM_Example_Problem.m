close all
clear all
clc
compute_metrics = false;
global use_reduced;
use_reduced = false;
global rollout_number;
rollout_number = 0;
global dont_print_time;
dont_print_time = true;

global test_dir;
test_dir = strcat('rollouts/',datestr(now));
mkdir(test_dir);

torque_limit = [100 80 60 40 20 10];
use_torque_constraint = true;
use_position_constraint = true;
tend = 8;
solver = 'euler';
dt = 0.01;
use_ellipsoid_regularization = false;
run('load_robot.m');

global robot;

eePositionTask = EETask(robot, 1.0, 10.0, 0.2);
elbowPositionTask = ElbowPositionTask(robot, 1.0, 10.0, 0.2);
jointPosTask = PostureTask(robot, 0.0001, 10.0, 0.2);


eePosRef = [-0.5675;   -0.2367;   -0.0144];
elPosRef = [-0.3229;   -0.0539;    0.2910];

eePositionTask.max_vel = 0.2;
elbowPositionTask.max_vel = 0.2;

eePositionTask.setDesired(eePosRef);
elbowPositionTask.setDesired(elPosRef);


tasks = {eePositionTask, elbowPositionTask, jointPosTask};
raw_data = Rollout(tasks, use_torque_constraint, use_position_constraint, torque_limit, compute_metrics, dt, tend, solver, use_ellipsoid_regularization);

rollout_data = RolloutData(raw_data);
save( strcat( test_dir, '/matlab_object-', int2str(rollout_number) ), 'rollout_data');

%%
% 
% n_samples = 8;
% step_size = size(rollout_data.task_ref_data{1,1},1) / n_samples;

middle_idx = round(size(rollout_data.task_ref_data{1,1},1) / 2);


global ee_times;
global ee_waypoints;
global el_times;
global el_waypoints;


ee_times = [rollout_data.task_ref_data{1,1}(1,1),rollout_data.task_ref_data{1,1}(300,1),rollout_data.task_ref_data{1,1}(600,1)];
ee_waypoints = [rollout_data.task_ref_data{1,2}(1,:)',rollout_data.task_ref_data{1,2}(300,:)',rollout_data.task_ref_data{1,2}(600,:)'];

el_times = [rollout_data.task_ref_data{2,1}(1,1),rollout_data.task_ref_data{2,1}(103,1),rollout_data.task_ref_data{2,1}(306,1)];
el_waypoints = [rollout_data.task_ref_data{2,2}(1,:)',rollout_data.task_ref_data{2,2}(103,:)',rollout_data.task_ref_data{2,2}(306,:)'];

% 
% el_times = rollout_data.task_ref_data{2,1}(1:step_size:end,1)';
% el_waypoints = rollout_data.task_ref_data{2,2}(1:step_size:end,:)';


global j_perf_0;
% theta_0 = [reshape(ee_waypoints(:,3:end-2),12,1); reshape(el_waypoints(:,3:end-2),12,1)];

theta_0 = [ee_waypoints(:,2); el_waypoints(:,2)];


j_perf_0 = rollout_data.performance_cost();


%%
% rollout_policy(theta_0)

options = struct('MaxIter',10, 'PopSize', 10);%,'TolFun',1e-3)
[xmin, fmin, counteval, stopflag, out, bestever] = cmaes('rollout_policy', theta_0, 2*var(theta_0), options);


%%
rollout_policy(xmin)

%%
save(strcat(test_dir, '/workspace'));