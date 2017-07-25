close all
clear all
clc;

example = 'temporally_incompatible_tasks';
d = load(strcat('./example_data_sets/matlab_object-',example));
rollout_data = d.rollout_data;
% rollout_data.parse_metric_data();
%%
switch example
    case 'incompatible_tasks'
        eePosRef = [0.7; -0.5; -0.1];
        elPosRef = [0.2; 0.5; 0.4];
    case 'temporally_incompatible_tasks'
        eePosRef = [-0.5675;   -0.2367;   -0.0144];
        elPosRef = [-0.3229;   -0.0539;    0.2910];
end

% rollout_data.animate();




rollout_data.animate(example);

%%
% rollout_data.plot_joint_positions();
% rollout_data.plot_joint_torques();                             
  
%%
% rollout_data.plot_constraint_evaluation_at_x_star(); 