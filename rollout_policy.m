function cost = rollout_policy(params)
global ee_times;
global ee_waypoints;
global el_times;
global el_waypoints;
global j_perf_0;
global rollout_number;
global robot;
global test_dir;

rollout_number = rollout_number + 1;
fprintf('Rollout No.:%i\t', rollout_number)
compute_metrics = false;

torque_limit = [100 80 60 40 20 10];
use_torque_constraint = true;
use_position_constraint = true;
tend = 8;
solver = 'euler';
dt = 0.01;
use_ellipsoid_regularization = false;

eePositionTask = EETask(robot, 1.0, 10.0, 0.2);
elbowPositionTask = ElbowPositionTask(robot, 1.0, 10.0, 0.2);
jointPosTask = PostureTask(robot, 0.0001, 10.0, 0.2);

% ee_wpts = ee_waypoints;
% ee_wpts(:,3:end-2) = reshape(params(1:12,1), 3, 4);
% 
% el_wpts = el_waypoints;
% el_wpts(:,3:end-2) = reshape(params(13:end,1), 3, 4);

ee_wpts = ee_waypoints;
ee_wpts(:,2) = params(1:3,1);

el_wpts = el_waypoints;
el_wpts(:,2) = params(4:end,1);

eePositionTask.max_vel = 0.2;
elbowPositionTask.max_vel = 0.2;

eePositionTask.setDesiredSpline(ee_times, ee_wpts);
elbowPositionTask.setDesiredSpline(el_times, el_wpts);


tasks = {eePositionTask, elbowPositionTask, jointPosTask};
raw_data = Rollout(tasks, use_torque_constraint, use_position_constraint, torque_limit, compute_metrics, dt, tend, solver, use_ellipsoid_regularization);

rollout_data = RolloutData(raw_data);
save( strcat( test_dir, '/matlab_object-', int2str(rollout_number) ), 'rollout_data');

cost = rollout_data.performance_cost() / j_perf_0;
fprintf('Cost:%2.3f\n', cost)

% rollout_data.animate()

end