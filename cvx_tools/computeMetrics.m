function [ compatibility_metrics, feasibility_metrics ] = computeMetrics(task_weights, Es, fs, optima, x_star, objective_ellipsoid, constraint_ellipsoid, controller_E, controller_f, A, b)
%COMPUTEMETRICS Summary of this function goes here
%   Detailed explanation goes here


compatibility_metrics.optima = optima;
compatibility_metrics.x_star = x_star;
optima_to_x_star = optima - repmat(x_star, 1, size(optima,2));
compatibility_metrics.distances = norms(optima_to_x_star, 2, 1);
compatibility_metrics.sum_distance = sum(compatibility_metrics.distances);

optima_to_obj_el_center = optima - repmat(objective_ellipsoid.center, 1, size(optima,2));
compatibility_metrics.center_distances = norms(optima_to_obj_el_center, 2, 1);
compatibility_metrics.sum_center_distance = sum(compatibility_metrics.center_distances);


n_objective_functions = size(optima,2);

compatibility_metrics.costs_at_x_star = zeros(1, n_objective_functions);
for i = 1:n_objective_functions
    compatibility_metrics.costs_at_x_star(1,i) = task_weights(i) * norm((Es{i}*x_star - fs{i}), 2)^2;
end

compatibility_metrics.sum_of_costs = sum(compatibility_metrics.costs_at_x_star);


n = size(optima,1);

compatibility_metrics.rank_A = rank(controller_E);
compatibility_metrics.rank_Ab = rank([controller_E, controller_f]);
compatibility_metrics.n = n;
compatibility_metrics.strictly_compatible = false;
if (compatibility_metrics.rank_A == compatibility_metrics.rank_Ab)
    if (compatibility_metrics.rank_Ab <= n)
        compatibility_metrics.strictly_compatible = true;
    end
end

compatibility_metrics.nuclear_norm_A = sum(svd(controller_E));
compatibility_metrics.nuclear_norm_Ab = sum(svd([controller_E, controller_f]));
compatibility_metrics.nuclear_norm_ratio = compatibility_metrics.nuclear_norm_A / compatibility_metrics.nuclear_norm_Ab;


E = [controller_E; A];
f = [controller_f; b];
compatibility_metrics.nuclear_norm_ratio_augmented = sum(svd(E)) / sum(svd([E,f]));


%%

optima_to_con_el_center = optima - repmat(constraint_ellipsoid.center, 1, size(optima,2));
feasibility_metrics.center_distances = norms(optima_to_con_el_center, 2, 1);
feasibility_metrics.sum_center_distance = sum(feasibility_metrics.center_distances);

feasibility_metrics.center_to_center_distance = norm((objective_ellipsoid.center - constraint_ellipsoid.center),2);

n_optima = size(optima,2);
feasibility_metrics.ellipsoid_inequality_measure = zeros(1,n_optima);
feasibility_metrics.optimum_is_in_ellipsoid = zeros(1,n_optima);

for i = 1:n_optima
    el_eval = (optima(:,i) - constraint_ellipsoid.d)' * constraint_ellipsoid.B^(-2) * (optima(:,i) - constraint_ellipsoid.d);
    feasibility_metrics.ellipsoid_inequality_measure(1,i) = el_eval;
    feasibility_metrics.optimum_is_in_ellipsoid(1,i) = el_eval <= 1.0;
    % Compute for the less conservative ellipsoid
    el_eval = (optima(:,i) - constraint_ellipsoid.d)' * (n*constraint_ellipsoid.B)^(-2) * (optima(:,i) - constraint_ellipsoid.d);
    feasibility_metrics.big_ellipsoid_inequality_measure(1,i) = el_eval;
    feasibility_metrics.optimum_is_in_big_ellipsoid(1,i) = el_eval <= 1.0;
end

feasibility_metrics.x_star_in_con_ellipsoid = (x_star - constraint_ellipsoid.d)' * constraint_ellipsoid.B^(-2) * (x_star - constraint_ellipsoid.d);

end

