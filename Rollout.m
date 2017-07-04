function rollout_data = Rollout(tasks, use_torque_constraint, use_position_constraint, torque_limit)

global robot;

global qn;


% initial state
y0 = [qn'; zeros(robot.n,1)];
%% Create controller
using_constraints = use_torque_constraint || use_position_constraint;
compute_metrics = true;

constraints = {};
if use_torque_constraint
    torqueConstraint = TorqueConstraint(robot, -torque_limit, torque_limit);
    constraints = [constraints, {torqueConstraint}];
end

if use_position_constraint
    positionConstraint = JointPositionConstraint(robot, robot.qlim(:,1), robot.qlim(:,2));
    constraints = [constraints, {positionConstraint}];
end



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

rollout_data = {robot, controller, t_traj, q_traj, tau_traj, tcp_traj, task_ref_data, step, torque_limit};
end




