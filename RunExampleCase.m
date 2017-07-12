run('load_robot.m');

% example = 'joint_positions_feasible';
% example = 'joint_positions_infeasible';
% example = 'compatible_tasks';
example = 'incompatible_tasks';
% example = 'instantaneously_compatible_tasks';
% example = 'instantaneously_incompatible_tasks';


eePositionTask = EETask(robot, 1.0, 10.0, 0.2);
elbowPositionTask = ElbowPositionTask(robot, 1.0, 10.0, 0.2);
tasks = {};
switch example
    case 'joint_positions_feasible'
        qn = qr;
        jointPosTask = PostureTask(robot, 1, 10.0, 0.2);
        tasks = {jointPosTask};

    case 'joint_positions_infeasible'
        qn = qr;
        jointPosTask = PostureTask(robot, 1, 10.0, 0.2);
        jointPosTask.max_vel = 0.4;
        jointPosTask.setDesired((robot.qlim(:,1)*1.1));
        tasks = {jointPosTask};
        
    case 'compatible_tasks'
        jointPosTask = PostureTask(robot, 0.0001, 10.0, 0.2);
        tasks = {eePositionTask, elbowPositionTask, jointPosTask};

    case 'incompatible_tasks'
        eePosRef = [0.7; -0.5; -0.1];
        elPosRef = [0.2; 0.5; 0.4];
        eePositionTask.setDesired(eePosRef);
        elbowPositionTask.setDesired(elPosRef);
        jointPosTask = PostureTask(robot, 0.0001, 10.0, 0.2);

        tasks = {eePositionTask, elbowPositionTask, jointPosTask};
end




torque_limit = 40;
use_torque_constraint = true; 
use_position_constraint = true;

raw_data = Rollout(tasks, use_torque_constraint, use_position_constraint, torque_limit);

%%
rollout_data = RolloutData(raw_data);

save(strcat('./example_data_sets/matlab_object-',example), 'rollout_data');

rollout_data.save_variables(strcat('./example_data_sets/',example));
   