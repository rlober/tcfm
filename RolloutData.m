classdef RolloutData
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

            obj.n_tasks = size(obj.controller.tasks,2);

            obj.robot_fig = figure();
            obj.robot_fig.Name = 'Robot Animation';
        end
        
        function visualize_task_references(obj)
            sphere_radius = 0.05;
            plot_sphere(eePosRef, sphere_radius, 'blue');
            plot_sphere(elPosRef, sphere_radius, 'red');
        end
        
        
        function animate(obj)
            %% Plot results
            
            disp('Displaying Animation')
            clf(obj.robot_fig);
            
%             obj.visualize_task_references();
            hold on;
            for i = 1:obj.n_tasks
                pos= obj.task_ref_data{i,2};
                Xs = pos(:,1);
                Ys = pos(:,2);
                Zs = pos(:,3);
                plot3(Xs,Ys,Zs);
            end
            hold off;
            
            obj.robot.plot(obj.q_traj, 'delay', obj.step);
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
                ylim([-obj.torque_limit; obj.torque_limit])
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
        

%         function parse_metric_data(obj)
        function plot_metric_data(obj)

        
            %% Parse metric data
            n_pts = size(obj.controller.metric_data,1);
            times = zeros(n_pts,1);
            c_sum_dist = zeros(n_pts,1);
            c_sum_cent_dist = zeros(n_pts,1);
            c_sum_costs = zeros(n_pts,1);
            f_sum_dist = zeros(n_pts,1);
            f_cen_to_cen_dist = zeros(n_pts,1);
            
            n_constraints = size(obj.controller.metric_data{1,7},1);
            GX = zeros(n_pts, n_constraints);
            H = zeros(n_pts, n_constraints);
            
            
            obj_el_volumes = zeros(n_pts,1);
            con_el_volumes = zeros(n_pts,1);
            
            n_max_objectives = 0;
            for i = 1:n_pts
                n_max_objectives = max([n_max_objectives, obj.controller.metric_data{i,6}]);
            end
            
            c_costs = zeros(n_pts, n_max_objectives);
            
            f_ellipsoid_inequality_measure = zeros(n_pts,n_max_objectives);
            f_optimum_is_in_ellipsoid = zeros(n_pts,n_max_objectives);
            f_x_star_in_con_ellipsoid = zeros(n_pts,1);
            
            for i = 1:n_pts
                times(i,1) = obj.controller.metric_data{i,1};
                cm = obj.controller.metric_data{i,2};
                c_sum_dist(i,1) = cm.sum_distance;
                c_sum_cent_dist(i,1) = cm.sum_center_distance;
                c_sum_costs(i,1) = cm.sum_of_costs;
                n_objectives = obj.controller.metric_data{i,6};
                c_costs(i,1:n_objectives) = cm.costs_at_x_star;
                
                fm = obj.controller.metric_data{i,3};
                f_sum_dist(i,1) = fm.sum_center_distance;
                f_cen_to_cen_dist(i,1) = fm.center_to_center_distance;
                f_ellipsoid_inequality_measure(i,1:n_objectives) = fm.ellipsoid_inequality_measure;
                f_optimum_is_in_ellipsoid(i,1:n_objectives) = fm.optimum_is_in_ellipsoid;
                f_x_star_in_con_ellipsoid(i,1) = fm.x_star_in_con_ellipsoid;
                
                obj_ellipsoid = obj.controller.metric_data{i,4};
                obj_el_volumes(i,1) = obj_ellipsoid.volume;
                
                con_ellipsoid = obj.controller.metric_data{i,5};
                con_el_volumes(i,1) = con_ellipsoid.volume;
                
                
                GX(i,:) = (obj.controller.metric_data{i,7}*obj.controller.metric_data{i,9})';
                H(i,:) = obj.controller.metric_data{i,8}';
            end
%         end
        
%         function plot_metric_data(obj)
            %% Plot metric data
            fig4 = figure();
            fig4.Name = 'Objective Compatibility Metrics';
            subplot(2,1,1)
            plot(times,c_sum_dist)
            ylabel('c sum dist')
            xlabel('t (sec)')
            subplot(2,1,2)
            plot(times,c_sum_cent_dist)
            ylabel('c sum cent dist')
            xlabel('t (sec)')
            
            
            fig4 = figure();
            fig4.Name = 'Objective Cost Metrics';
            
            for i = 1:n_max_objectives
                subplot(n_max_objectives+1,1,i)
                plot(times,c_costs(:,i))
                ylabel(sprintf('cost f_%i',i))
                xlabel('t (sec)')
            end
            subplot(n_max_objectives+1,1,n_max_objectives+1)
            plot(times,c_sum_costs)
            ylabel('c sum costs')
            xlabel('t (sec)')
            
            fig5 = figure();
            fig5.Name = 'Objective Feasibility Metrics';
            subplot(2,1,1)
            plot(times,f_sum_dist)
            ylabel('f sum dist')
            xlabel('t (sec)')
            
            subplot(2,1,2)
            plot(times,f_cen_to_cen_dist)
            ylabel('f cen to cen dist')
            xlabel('t (sec)')
            
            fig6 = figure();
            fig6.Name = 'Ellipsoid Volumes';
            subplot(2,1,1)
            plot(times,obj_el_volumes)
            ylabel('obj el volumes')
            xlabel('t (sec)')
            
            subplot(2,1,2)
            plot(times,con_el_volumes)
            ylabel('con el volumes')
            xlabel('t (sec)')
            
            fig7 = figure();
            fig7.Name = 'Ellipsoid Equation Evaluation';
            for i = 1:n_max_objectives
                subplot(n_max_objectives+1,1,i)
                plot(times,f_ellipsoid_inequality_measure(:,i))
                ylabel(sprintf('ellipsoid inequality measure f_%i',i))
                xlabel('t (sec)')
            end
            subplot(n_max_objectives+1,1,n_max_objectives+1)
            plot(times,f_x_star_in_con_ellipsoid)
            ylabel('f x^* in con ellipsoid')
            xlabel('t (sec)')
            
            
            n_con_types = size(obj.controller.constraints,2);
            n_con_eqns = obj.robot.n * 2;
            k = 1;
            for j = 1:n_con_types
                
                fig8 = figure();
                fig8.Name = 'Constraint Evaluation at x^*';
                m = 1;
                for i = k:n_con_eqns*j
                    subplot(2, (n_con_eqns/2), m)
                    plot(times, H(:,i), 'Color', [0.6, 0.6, 0], 'LineWidth', 3);
                    hold on;
                    plot(times, GX(:,i), 'Color', [0.0, 0.6, 0], 'LineWidth', 3);
                    for f = 1:size(H,1)
                        if ( GX(f,i) > H(f,i) )
                            plot(times(f,1), GX(f,i), 'LineStyle', 'none', 'Marker', 'o', 'MarkerFaceColor', [0.6, 0, 0], 'MarkerSize', 4)
                        end
                    end
                    
                    hold off;
                    m = m+1;
                end
                k = k+n_con_eqns;
            end
        end
    end
end

    

