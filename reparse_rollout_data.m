close all
clear all
clc

test_examples = {};

test_examples = [test_examples, 'joint_positions_feasible'];
test_examples = [test_examples, 'joint_positions_infeasible'];
test_examples = [test_examples, 'joint_positions_infeasible_with_trajectory'];

% test_examples = [test_examples, 'compatible_tasks'];
% test_examples = [test_examples, 'incompatible_tasks'];
% test_examples = [test_examples, 'incompatible_tasks_with_trajectory'];

% test_examples = [test_examples, 'temporally_incompatible_tasks'];
for i = 1:size(test_examples,2)
    example = test_examples{i};
    rd = load(strcat('./example_data_sets/matlab_object-',example));
    rd.rollout_data.parse_metric_data();
    rd.rollout_data.save_variables(strcat('./example_data_sets/',example)); 
end