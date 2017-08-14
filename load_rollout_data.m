close all
clear all
clc

comp = load('./example_data_sets/matlab_object-compatible_tasks');
incomp = load('./example_data_sets/matlab_object-incompatible_tasks');
traj_incomp = load('./example_data_sets/matlab_object-incompatible_tasks_with_trajectory');


feas = load('./example_data_sets/matlab_object-joint_positions_feasible');
infeas = load('./example_data_sets/matlab_object-joint_positions_infeasible');
traj_infeas = load('./example_data_sets/matlab_object-joint_positions_infeasible_with_trajectory');

temporal = load('./example_data_sets/matlab_object-temporally_incompatible_tasks');
