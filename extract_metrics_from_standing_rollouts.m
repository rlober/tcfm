comp = {};
feas = {};
cf_times = {};
% ocnr = zeros(size(time));
% ocnr_A = zeros(size(time));
for i = 1:10:size(time,1)
    fprintf('%f percent finished\n', i/size(time,1)*100)
    var_dim = size(controller{i,5},2);
    Es = task_E(i,:);
    fs = task_f(i,:);
    A = controller{i,1};
    b = controller{i,2}';
    G = controller{i,3};
    h = controller{i,4}';
    weights = task_weights(i,:);

    optima = zeros(var_dim,size(Es,2));
    ctrl_E = [];
    ctrl_f = [];
    ws = [];
    for j = 1:size(Es,2)
        [m,n] = size(Es{j});
        Es{j} = [Es{j}, zeros(m, var_dim-n)];
        fs{j} = fs{j}';
        optima(:,j) = CLS(Es{j}, fs{j}, A, b);
        ctrl_E = [ctrl_E; Es{j}];
        ctrl_f = [ctrl_f; fs{j}];
        ws = [ws, mean(weights{j})];
    end
    x_star = CLS(ctrl_E, ctrl_f, A, b);

%     nuclear_norm_A = sum(svd(ctrl_E));
%     nuclear_norm_Ab = sum(svd([ctrl_E, ctrl_f]));
%     ocnr(i,1) = nuclear_norm_A / nuclear_norm_Ab;
% 
% 
%     E = [ctrl_E; A];
%     f = [ctrl_f; b];
%     ocnr_A(i,1) = sum(svd(E)) / sum(svd([E,f]));

    [obj_el, con_el] = computeConstraintAndObjectiveEllipses(optima, G, h, 0);
    [ compatibility_metrics, feasibility_metrics ] = computeMetrics(ws, Es, fs, optima, x_star, obj_el, con_el, ctrl_E, ctrl_f, A, b);
    
    comp = [comp; compatibility_metrics];
    feas = [feas; feasibility_metrics];
    cf_times = [cf_times; time(i,1)];
end

%%
save('example_data_sets/standing_data/original/comp_feas_metrics.mat', 'comp', 'feas', 'cf_times');

%%
% 
% fig1 = figure();
% % fig1.Title = 'Nuclear Norm Ratio';
% plot(time, ocnr)
% fig2 = figure();
% % fig2.Title = 'Augmented Nuclear Norm Ratio';
% plot(time, ocnr_A)