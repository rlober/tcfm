run('load_robot.m');

example = 'joint_positions_feasible';
% example = 'joint_positions_infeasible';
% example = 'instantaneously_compatible_tasks';
% example = 'instantaneously_incompatible_tasks';
% example = 'instantaneously_compatible_tasks';
% example = 'instantaneously_incompatible_tasks';


eePositionTask = EETask(robot, 1.0, 10.0, 0.2);
elbowPositionTask = ElbowPositionTask(robot, 1.0, 10.0, 0.2);
jointPosTask = PostureTask(robot, 0.0001, 10.0, 0.2);
tasks = {};
switch example
    case 'joint_positions_feasible'
        qn = qr;
        tasks = {jointPosTask};

    case 'joint_positions_infeasible'
        qn = qr;
        jointPosTask.max_vel = 0.4;
        jointPosTask.setDesired((robot.qlim(:,1)*1.1));
        tasks = {jointPosTask};
        
    case 'instantaneously_compatible_tasks'
        tasks = {eePositionTask, elbowPositionTask, jointPosTask};

    case 'instantaneously_incompatible_tasks'
        eePosRef = [-0.25; -0.5; -0.22];
        eePositionTask.setDesired(eePosRef);
        elbowPositionTask.setDesired(eePosRef);
        tasks = {eePositionTask, elbowPositionTask, jointPosTask};
end




torque_limit = 40;
use_torque_constraint = true; 
use_position_constraint = true;

raw_data = Rollout(tasks, use_torque_constraint, use_position_constraint, torque_limit);

%%
rollout_data = RolloutData(raw_data);

close all;
save(strcat('./example_data_sets/',example), 'rollout_data');
% 
% %%
% rollout_data.animate();
% 
% %%
% rollout_data.plot_joint_positions();
% rollout_data.plot_joint_torques();                             
%   
% %%
% rollout_data.plot_metric_data();    