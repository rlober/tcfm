clear all;
clc;
base_dir='~/tcfm_metric_test_data/';
dirs = dir(base_dir);
n_dirs = size(dirs,1);

time = [];

controller = {};




for i = 1:n_dirs
    dir_path = strcat(base_dir, int2str(i));
    try
        t = load( strcat(dir_path, '/', 'time.txt') );
    catch
        fprintf('No time file found for directory number: %i\n', i)
        break;
    end
   
    G = load( strcat(dir_path, '/', 'One Level Solver with QuadProg++ subSolver_G.txt') );
    h = load( strcat(dir_path, '/', 'One Level Solver with QuadProg++ subSolver_h.txt') );
    
    
    
    time = [time; t];

    controller = [controller; {G, h}];
    
   
end

%%
feas_ellipsoids = {};
for i = 1%:size(time,1)
    G = controller{i,1};
    h = controller{i,2}';
    [m,n] = size(G);
    cvx_begin
        variable B(n,n) symmetric
        variable d(n)
        maximize( det_rootn( B ) )
        subject to
        for i = 1:m
            norm( B*G(i,:)', 2 ) + G(i,:)*d <= h(i);
        end
    cvx_end
    
    constraint_ellipsoid.B = B;
    constraint_ellipsoid.d = d;
    constraint_ellipsoid.center = d;
    
    try
        [constraint_ellipsoid.U,constraint_ellipsoid.S,constraint_ellipsoid.V] = svd(constraint_ellipsoid.B);
        constraint_ellipsoid.US = constraint_ellipsoid.U*constraint_ellipsoid.S;
        constraint_ellipsoid.volume = det(constraint_ellipsoid.S);
        
        constraint_ellipsoid.radii = {};
        for i = 1:size(constraint_ellipsoid.US,2)
            point = constraint_ellipsoid.center - constraint_ellipsoid.US(:,i);
            radii = [constraint_ellipsoid.center, point];
            constraint_ellipsoid.radii = [constraint_ellipsoid.radii, radii];
        end
        
    catch
        warning('Could not compute SVD of constraint ellipsoid');
        constraint_ellipsoid.U = zeros(n,n);
        constraint_ellipsoid.S = zeros(n,n);
        constraint_ellipsoid.V = zeros(n,n);
        constraint_ellipsoid.US = zeros(n,n);
        constraint_ellipsoid.volume = 0;
        constraint_ellipsoid.radii = {};
        
        for i = 1:n
            constraint_ellipsoid.radii =  [constraint_ellipsoid.radii, {zeros(n,1), zeros(n,1)}];
        end
        
    end
    
    feas_ellipsoids = [feas_ellipsoids; constraint_ellipsoid];
end