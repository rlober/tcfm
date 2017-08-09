close all
clear all
clc
compute_metrics = true;
global use_reduced;
use_reduced = false;

test_examples = {};
% test_examples = [test_examples, 'joint_positions_feasible'];
% test_examples = [test_examples, 'joint_positions_infeasible'];
test_examples = [test_examples, 'compatible_tasks'];
test_examples = [test_examples, 'incompatible_tasks'];
test_examples = [test_examples, 'incompatible_tasks_with_trajectory'];
% test_examples = [test_examples, 'temporally_incompatible_tasks'];

for i = 1:size(test_examples,2)
    clearvars -except test_examples i compute_metrics use_reduced
    clc;
    example = test_examples{i};
    run('load_robot.m');
    sprintf('running example: %s', example)
    eePositionTask = EETask(robot, 1.0, 10.0, 0.2);
    elbowPositionTask = ElbowPositionTask(robot, 1.0, 10.0, 0.2);
    tasks = {};
    
    solver = 'euler';
    dt = 0.01;
    tend = 4;
    
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
            eePositionTask.setReferences(eePosRef, [], []);
            elbowPositionTask.setReferences(elPosRef, [], []);
            jointPosTask = PostureTask(robot, 0.0001, 10.0, 0.2);
            
            tasks = {eePositionTask, elbowPositionTask, jointPosTask};
            
        case 'incompatible_tasks_with_trajectory'
            eePosRef = [0.7; -0.5; -0.1];
            elPosRef = [0.2; 0.5; 0.4];
            eePositionTask.setDesired(eePosRef);
            elbowPositionTask.setDesired(elPosRef);
            jointPosTask = PostureTask(robot, 0.0001, 10.0, 0.2);
            
            tasks = {eePositionTask, elbowPositionTask, jointPosTask};
            
        case 'temporally_incompatible_tasks'
            eePosRef = [-0.5675;   -0.2367;   -0.0144];
            elPosRef = [-0.3229;   -0.0539;    0.2910];
            %         eePosRef = [-0.3846;   -0.4798;   -0.0144];
            %         elPosRef = [-0.2575;   -0.2021;    0.2910];
            eePositionTask.max_vel = 0.2;
            elbowPositionTask.max_vel = 0.2;
            
            eePositionTask.setDesired(eePosRef);
            elbowPositionTask.setDesired(elPosRef);
            jointPosTask = PostureTask(robot, 0.0001, 10.0, 0.2);
            
            tasks = {eePositionTask, elbowPositionTask, jointPosTask};
    end
    
    
    
    
    torque_limit = [100 80 60 40 20 10];
    use_torque_constraint = true;
    use_position_constraint = true;
    
    raw_data = Rollout(tasks, use_torque_constraint, use_position_constraint, torque_limit, compute_metrics, dt, tend, solver);
    
    %%
    rollout_data = RolloutData(raw_data);
    
    save(strcat('./example_data_sets/matlab_object-',example), 'rollout_data');
    
    rollout_data.save_variables(strcat('./example_data_sets/',example));
end