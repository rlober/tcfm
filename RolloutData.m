classdef RolloutData < handle
    %ROLLOUTDATA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        data
        robot
        controller
        t_traj
        q_traj
        tau_traj
        tcp_traj
        task_ref_data
        step
        n_tasks
        torque_limit
        
        times
        c_sum_dist
        c_sum_cent_dist
        c_sum_costs
        f_sum_dist 
        f_cen_to_cen_dist 
        GX 
        H 
        obj_el_volumes 
        con_el_volumes 
        c_costs
        f_ellipsoid_inequality_measure
        f_center_distances
        f_big_ellipsoid_inequality_measure
        f_optimum_is_in_ellipsoid
        f_x_star_in_con_ellipsoid
        
        rank_A
        rank_Ab
        n
        strictly_compatible
        nuclear_norm_A
        nuclear_norm_Ab
        nuclear_norm_ratio
        nuclear_norm_ratio_augmented
        q_upper
        q_lower
        
        n_max_objectives
        
        task_acc_des_norms
        c_distances
        c_center_distances
        
        c_x_star_to_obj_el_center
        
        optima_norms;
        x_star_norms;
            
        robot_fig
    end
    
    methods
        function obj = RolloutData(rollout_data)
            obj.data = rollout_data;
            obj.robot = obj.data{1};
            obj.controller = obj.data{2};
            obj.t_traj = obj.data{3};
            obj.q_traj = obj.data{4};
            obj.tau_traj = obj.data{5};
            obj.tcp_traj = obj.data{6};
            obj.task_ref_data = obj.data{7};
            obj.step = obj.data{8};
            obj.torque_limit = obj.data{9};
            obj.task_acc_des_norms = obj.data{10};
            obj.q_lower = obj.robot.qlim(:,1);
            obj.q_upper = obj.robot.qlim(:,2);

            obj.n_tasks = size(obj.controller.tasks,2);
            try
                obj.parse_metric_data();
            catch
                disp('could not parse metric data')
            end
            
        end
        
        function visualize_task_references(obj)
            sphere_radius = 0.05;
%             eePosRef = [0.7; -0.5; -0.1];
%             elPosRef = [0.2; 0.5; 0.4];
            eePosRef = [-0.5675;   -0.2367;   -0.0144];
            elPosRef = [-0.3229;   -0.0539;    0.2910];
            plot_sphere(eePosRef, sphere_radius, 'blue');
            plot_sphere(elPosRef, sphere_radius, 'red');
        end
        
        function save_variables(obj, filename)
            t_traj=obj.t_traj;
            q_traj=obj.q_traj;
            tau_traj=obj.tau_traj;
            tcp_traj=obj.tcp_traj;
            task_ref_data=obj.task_ref_data;
            step=obj.step;
            n_tasks=obj.n_tasks;
            torque_limit=obj.torque_limit;
            times=obj.times;
            c_sum_dist=obj.c_sum_dist;
            c_sum_cent_dist=obj.c_sum_cent_dist;
            c_sum_costs=obj.c_sum_costs;
            f_sum_dist =obj.f_sum_dist;
            f_cen_to_cen_dist =obj.f_cen_to_cen_dist;
            GX =obj.GX;
            H =obj.H;
            obj_el_volumes =obj.obj_el_volumes;
            con_el_volumes =obj.con_el_volumes;
            c_costs=obj.c_costs;
            f_ellipsoid_inequality_measure=obj.f_ellipsoid_inequality_measure;
            f_optimum_is_in_ellipsoid=obj.f_optimum_is_in_ellipsoid;
            f_x_star_in_con_ellipsoid=obj.f_x_star_in_con_ellipsoid;       
            q_upper = obj.q_upper;
            q_lower = obj.q_lower;
            f_big_ellipsoid_inequality_measure=obj.f_big_ellipsoid_inequality_measure;
            rank_A = obj.rank_A;
            rank_Ab = obj.rank_Ab;
            n = obj.n;
            strictly_compatible = obj.strictly_compatible;
            nuclear_norm_A = obj.nuclear_norm_A;
            nuclear_norm_Ab = obj.nuclear_norm_Ab;
            nuclear_norm_ratio = obj.nuclear_norm_ratio;
            nuclear_norm_ratio_augmented = obj.nuclear_norm_ratio_augmented;
            task_acc_des_norms = obj.task_acc_des_norms;
            c_distances = obj.c_distances;
            c_center_distances = obj.c_center_distances;
            c_x_star_to_obj_el_center = obj.c_x_star_to_obj_el_center;
            optima_norms = obj.optima_norms;
            x_star_norms = obj.x_star_norms;
            f_center_distances = obj.f_center_distances;
            save(filename, 't_traj', 'q_traj', 'tau_traj', 'tcp_traj', 'task_ref_data', 'step', 'n_tasks', 'torque_limit', 'times', 'c_sum_dist', 'c_sum_cent_dist', 'c_sum_costs', 'f_sum_dist', 'f_cen_to_cen_dist', 'GX', 'H', 'obj_el_volumes', 'con_el_volumes', 'c_costs', 'f_ellipsoid_inequality_measure', 'f_optimum_is_in_ellipsoid', 'f_x_star_in_con_ellipsoid', 'q_upper', 'q_lower', 'f_big_ellipsoid_inequality_measure', 'rank_A', 'rank_Ab', 'n', 'strictly_compatible', 'nuclear_norm_A', 'nuclear_norm_Ab', 'nuclear_norm_ratio', 'nuclear_norm_ratio_augmented', 'task_acc_des_norms', 'c_distances', 'c_center_distances', 'c_x_star_to_obj_el_center', 'optima_norms', 'x_star_norms', 'f_center_distances');
        end
        
        function animate(obj, movie_name)
            %% Plot results
            
            disp('Displaying Animation')
            try
                clf(obj.robot_fig);
            catch
                obj.robot_fig = figure();
                obj.robot_fig.Name = 'Robot Animation';
            end
            
            obj.visualize_task_references();
            
            hold on;
            for i = 1:obj.n_tasks
                pos= obj.task_ref_data{i,2};
                Xs = pos(:,1);
                Ys = pos(:,2);
                Zs = pos(:,3);
                plot3(Xs,Ys,Zs);
            end
            hold off;
            if exist('movie_name', 'var')
                obj.robot.plot(obj.q_traj, 'delay', obj.step, 'movie', movie_name);
            else
                obj.robot.plot(obj.q_traj, 'delay', obj.step);
            end
            disp('Animation Finished')
        end
        
        
        function plot_joint_positions(obj)
            %% Plot Joint Positions and Torques
            fig1 = figure();
            fig1.Name = 'Joint Position Curves';
            rad_to_deg = (180/pi);
            q_traj_deg = rad_to_deg*obj.q_traj;
            qlims_deg = rad_to_deg*obj.robot.qlim;
            for i = 1:obj.robot.n
                subplot(2,3,i)
                plot(obj.t_traj, q_traj_deg(:,i), 'b')
                ylim([qlims_deg(i,1), qlims_deg(i,2)])
                ylabel('joint position (degree)')
                xlabel('t (sec)')
                title(sprintf('Joint %i',i))
            end
        end

        function plot_joint_torques(obj)

            fig2 = figure();
            fig2.Name = 'Torque Curves';
            for i = 1:obj.robot.n
                subplot(2,3,i)
                plot(obj.t_traj, obj.tau_traj(:,i), 'b')
                if size(obj.torque_limit,2) > 1
                    ylim([-obj.torque_limit(1,i); obj.torque_limit(1,i)])
                else
                    ylim([-obj.torque_limit; obj.torque_limit])
                end
                ylabel('tau (Nm)')
                xlabel('t (sec)')
                title(sprintf('Joint %i',i))
            end

        end

        function plot_task_references(obj)
            %% Plot task references
            for i = 1:obj.n_tasks
                
                tmp_times = obj.task_ref_data{i,1};
                tmp_pos = obj.task_ref_data{i,2};
                tmp_vel = obj.task_ref_data{i,3};
                tmp_acc = obj.task_ref_data{i,4};
                n_dof = size(tmp_pos,2);
                
                fig3 = figure();
                fig3.Name = sprintf('Task %i reference trajectory',i);
                for j = 1:n_dof
                    
                    subplot(3, n_dof, j);
                    plot(tmp_times, tmp_pos(:,j))
                    xlabel('t (sec)')
                    ylabel('pos (m)')
                    
                    subplot(3, n_dof, j+n_dof);
                    plot(tmp_times, tmp_vel(:,j))
                    xlabel('t (sec)')
                    ylabel('vel (m/s)')
                    
                    subplot(3, n_dof, j+2*n_dof);
                    plot(tmp_times, tmp_acc(:,j))
                    xlabel('t (sec)')
                    ylabel('acc (m/s^2)')
                    
                end
            end
        end
        

        function parse_metric_data(obj)
            n_pts = size(obj.controller.metric_data,1);
            obj.times = zeros(n_pts,1);
            obj.c_sum_dist = zeros(n_pts,1);
            obj.c_sum_cent_dist = zeros(n_pts,1);
            obj.c_sum_costs = zeros(n_pts,1);
            obj.f_sum_dist = zeros(n_pts,1);
            obj.f_cen_to_cen_dist = zeros(n_pts,1);
            
            obj.c_x_star_to_obj_el_center = zeros(n_pts,1);

            
            n_constraints = size(obj.controller.metric_data{1,7},1);
            obj.GX = zeros(n_pts, n_constraints);
            obj.H = zeros(n_pts, n_constraints);
            
            
            obj.obj_el_volumes = zeros(n_pts,1);
            obj.con_el_volumes = zeros(n_pts,1);
            
            obj.n_max_objectives = 0;
            for i = 1:n_pts
                obj.n_max_objectives = max([obj.n_max_objectives, obj.controller.metric_data{i,6}]);
            end
            
            obj.optima_norms = zeros(n_pts, obj.n_max_objectives);
            obj.x_star_norms = zeros(n_pts,1);
            
            
            obj.c_costs = zeros(n_pts, obj.n_max_objectives);
            obj.c_distances = zeros(n_pts, obj.n_max_objectives);
            obj.c_center_distances = zeros(n_pts, obj.n_max_objectives);
            
            obj.f_ellipsoid_inequality_measure = zeros(n_pts,obj.n_max_objectives);
            obj.f_center_distances = zeros(n_pts,obj.n_max_objectives);

            obj.f_big_ellipsoid_inequality_measure = zeros(n_pts,obj.n_max_objectives);
            obj.f_optimum_is_in_ellipsoid = zeros(n_pts,obj.n_max_objectives);
            obj.f_x_star_in_con_ellipsoid = zeros(n_pts,1);
            
            obj.rank_A = zeros(n_pts,1);
            obj.rank_Ab = zeros(n_pts,1);
            obj.n = zeros(n_pts,1);
            obj.strictly_compatible = zeros(n_pts,1);
            obj.nuclear_norm_A = zeros(n_pts,1);
            obj.nuclear_norm_Ab = zeros(n_pts,1);
            obj.nuclear_norm_ratio = zeros(n_pts,1);
            obj.nuclear_norm_ratio_augmented = zeros(n_pts,1);
            
            for i = 1:n_pts
                obj.times(i,1) = obj.controller.metric_data{i,1};
                cm = obj.controller.metric_data{i,2};
                obj.c_sum_dist(i,1) = cm.sum_distance;
                obj.c_sum_cent_dist(i,1) = cm.sum_center_distance;
                obj.c_sum_costs(i,1) = cm.sum_of_costs;
                
                obj.c_x_star_to_obj_el_center(i,1) = norm( obj.controller.metric_data{i, 4}.center  - obj.controller.metric_data{i, 9} );
                
                n_optima = size(obj.controller.metric_data{i, 2}.optima,2);
                for k = 1:n_optima
                    obj.optima_norms(i,k) = norm(obj.controller.metric_data{i, 2}.optima(:,k));
                end
                obj.x_star_norms(i,1) = norm(obj.controller.metric_data{i, 2}.x_star);
                
                n_objectives = obj.controller.metric_data{i,6};
                obj.c_costs(i,1:n_objectives) = cm.costs_at_x_star;
                obj.c_distances(i,1:n_objectives) = cm.distances;
                obj.c_center_distances(i,1:n_objectives) = cm.center_distances;
                
                obj.rank_A(i,1) = cm.rank_A;
                obj.rank_Ab(i,1) = cm.rank_Ab;
                obj.n(i,1) = cm.n;
                obj.strictly_compatible(i,1) = cm.strictly_compatible;
                obj.nuclear_norm_A(i,1) = cm.nuclear_norm_A;
                obj.nuclear_norm_Ab(i,1) = cm.nuclear_norm_Ab;
                obj.nuclear_norm_ratio(i,1) = cm.nuclear_norm_ratio;
                obj.nuclear_norm_ratio_augmented(i,1) = cm.nuclear_norm_ratio_augmented;
                
                fm = obj.controller.metric_data{i,3};
                obj.f_center_distances(i,1:n_objectives) = fm.center_distances;
                obj.f_sum_dist(i,1) = fm.sum_center_distance;
                obj.f_cen_to_cen_dist(i,1) = fm.center_to_center_distance;
                obj.f_ellipsoid_inequality_measure(i,1:n_objectives) = fm.ellipsoid_inequality_measure;
                obj.f_big_ellipsoid_inequality_measure(i,1:n_objectives) = fm.big_ellipsoid_inequality_measure;
                obj.f_optimum_is_in_ellipsoid(i,1:n_objectives) = fm.optimum_is_in_ellipsoid;
                obj.f_x_star_in_con_ellipsoid(i,1) = fm.x_star_in_con_ellipsoid;
                
                obj_ellipsoid = obj.controller.metric_data{i,4};
                obj.obj_el_volumes(i,1) = obj_ellipsoid.volume;
                
                con_ellipsoid = obj.controller.metric_data{i,5};
                obj.con_el_volumes(i,1) = con_ellipsoid.volume;
                
                
                obj.GX(i,:) = (obj.controller.metric_data{i,7}*obj.controller.metric_data{i,9})';
                obj.H(i,:) = obj.controller.metric_data{i,8}';
            end
        end
        
        function plot_compatibility_distance_metric(obj)
            %% Plot metric data
            fig4 = figure();
            fig4.Name = 'Objective Compatibility Metrics';
            subplot(2,1,1)
            plot(obj.times,obj.c_sum_dist)
            ylabel('c sum dist')
            xlabel('t (sec)')
            subplot(2,1,2)
            plot(obj.times,obj.c_sum_cent_dist)
            ylabel('c sum cent dist')
            xlabel('t (sec)')
        end
        
        function plot_compatibility_cost_metric(obj)
            fig4 = figure();
            fig4.Name = 'Objective Cost Metrics';
            
            for i = 1:obj.n_max_objectives
                subplot(obj.n_max_objectives+1,1,i)
                plot(obj.times,obj.c_costs(:,i))
                ylabel(sprintf('cost f_%i',i))
                xlabel('t (sec)')
            end
            subplot(obj.n_max_objectives+1,1,obj.n_max_objectives+1)
            plot(obj.times,obj.c_sum_costs)
            ylabel('c sum costs')
            xlabel('t (sec)')
        end
        
        function plot_feasibility_distance_metric(obj)

            fig5 = figure();
            fig5.Name = 'Objective Feasibility Metrics';
            subplot(2,1,1)
            plot(obj.times,obj.f_sum_dist)
            ylabel('f sum dist')
            xlabel('t (sec)')
            
            subplot(2,1,2)
            plot(obj.times,obj.f_cen_to_cen_dist)
            ylabel('f cen to cen dist')
            xlabel('t (sec)')
        end
        
        function plot_ellipsoid_volumes(obj)

            fig6 = figure();
            fig6.Name = 'Ellipsoid Volumes';
            subplot(2,1,1)
            plot(obj.times,obj.obj_el_volumes)
            ylabel('obj el volumes')
            xlabel('t (sec)')
            
            subplot(2,1,2)
            plot(obj.times,obj.con_el_volumes)
            ylabel('con el volumes')
            xlabel('t (sec)')
        end
        
        function plot_constraint_ellipsoid_evaluation_at_x_star(obj)

            fig7 = figure();
            fig7.Name = 'Ellipsoid Equation Evaluation';
            for i = 1:obj.n_max_objectives
                subplot(obj.n_max_objectives+1,1,i)
                plot(obj.times,obj.f_ellipsoid_inequality_measure(:,i))
                ylabel(sprintf('ellipsoid inequality measure f_%i',i))
                xlabel('t (sec)')
            end
            subplot(obj.n_max_objectives+1,1,obj.n_max_objectives+1)
            plot(obj.times,obj.f_x_star_in_con_ellipsoid)
            ylabel('f x^* in con ellipsoid')
            xlabel('t (sec)')
            
        end
        
        function plot_constraint_evaluation_at_x_star(obj)

            n_con_types = size(obj.controller.constraints,2);
            n_con_eqns = obj.robot.n * 2;
            current_constraint_index = 1;
            fig8 = figure();
            fig8.Name = 'Constraint Evaluation at x^*';
                m = 1;
                for i = 1:n_con_eqns*n_con_types
                    
                    subplot(n_con_types, (n_con_eqns), m)
                    plot(obj.times, obj.H(:,i), 'Color', [0.6, 0.6, 0], 'LineWidth', 3);
                    hold on;
                    plot(obj.times, obj.GX(:,i), 'Color', [0.0, 0.6, 0], 'LineWidth', 3);
                    for f = 1:size(obj.H,1)
                        if ( obj.GX(f,i) > obj.H(f,i) )
                            plot(obj.times(f,1), obj.GX(f,i), 'LineStyle', 'none', 'Marker', 'o', 'MarkerFaceColor', [0.6, 0, 0], 'MarkerSize', 4)
                        end
                    end
                    
                    if (i == 1 || mod(i,n_con_eqns+1)==0) && current_constraint_index <= n_con_types
                        ylabel(class(obj.controller.constraints{current_constraint_index}));
                        current_constraint_index = current_constraint_index + 1;
                    end
                    
                    hold off;
                    m = m+1;
                end
            leg1 = legend('$h$', '$Gx^*$', '$Gx^* > h$');
            set(leg1,'Interpreter','latex');
            set(leg1,'FontSize',17);
            set(leg1,'Location','northwestoutside');
        end
    end
end

    

