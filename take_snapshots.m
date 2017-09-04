close all
clear all
clc

test_examples = {};

test_examples = [test_examples, 'joint_positions_feasible'];
test_examples = [test_examples, 'joint_positions_infeasible'];
test_examples = [test_examples, 'joint_positions_infeasible_with_trajectory'];

test_examples = [test_examples, 'compatible_tasks'];
test_examples = [test_examples, 'incompatible_tasks'];
test_examples = [test_examples, 'incompatible_tasks_with_trajectory'];

test_examples = [test_examples, 'temporally_incompatible_tasks'];

test_examples = [test_examples, 'temporally_incompatible_tasks_mf_comp_feas'];


for i = 1:size(test_examples,2)
    example = test_examples{i};
    rd = load(strcat('./example_data_sets/matlab_object-',example));
    rd.rollout_data.parse_metric_data();
    if i > 3
        rd.rollout_data.take_snapshots(example, rd.rollout_data.controller.tasks{1,1}.pos_ref, rd.rollout_data.controller.tasks{1,2}.pos_ref); 
    else
        rd.rollout_data.take_snapshots(example, [], []); 
    end
end