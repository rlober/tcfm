% rd = d0.rollout_data;
% ee_times = [rd.task_ref_data{1,1}(1,1),rd.task_ref_data{1,1}(300,1),rd.task_ref_data{1,1}(600,1)]
% ee_waypoints = [rd.task_ref_data{1,2}(1,:)',rd.task_ref_data{1,2}(300,:)',rd.task_ref_data{1,2}(600,:)']
% 
% el_times = [rd.task_ref_data{2,1}(1,1),rd.task_ref_data{2,1}(103,1),rd.task_ref_data{2,1}(306,1)]
% el_waypoints = [rd.task_ref_data{2,2}(1,:)',rd.task_ref_data{2,2}(103,:)',rd.task_ref_data{2,2}(306,:)']


j_trac = [];
j_goal = [];
j_tp = [];
j_energy = [];
j_perf = [];

for i = 0:104
    
d = load(sprintf('rollouts/27-Sep-2017 11:49:55/matlab_object-%i.mat',i));
    
d.rollout_data.performance_cost();
j_trac = [j_trac; d.rollout_data.j_trac];
j_goal = [j_goal; d.rollout_data.j_goal];
j_tp = [j_tp; d.rollout_data.j_tp];
j_energy = [j_energy; d.rollout_data.j_energy];
j_perf = [j_perf; d.rollout_data.j_perf];

end

save('./example_data_sets/tcfm_cost_data', 'j_trac', 'j_goal', 'j_tp', 'j_energy', 'j_perf');