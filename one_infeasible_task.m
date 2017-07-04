close all;
clear all; 
clc;
%% Load model
mdl_puma560;
global robot;
robot = p560;

global qn;
qn(1) =  3*pi/4;
qn(3) = -pi;

eePosRef = [-0.25; -0.5; -0.22];

eePositionTask = EETask(robot, 1.0, 10.0, 0.2);
eePositionTask.setDesired(eePosRef);

tasks = {eePositionTask};
torque_limit = 40;
use_torque_constraint = true; 
use_position_constraint = true;

raw_data = Rollout(tasks, use_torque_constraint, use_position_constraint, torque_limit);

rollout_data = RolloutData(raw_data);

rollout_data.animate();
