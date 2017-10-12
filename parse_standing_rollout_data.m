clear all;
clc;
% base_dir='~/tcfm_metric_test_data/';
base_dir='~/standing_original/';
dirs = dir(base_dir);
n_dirs = size(dirs,1);

time = [];
task_E = {};
task_f = {};
controller = {};
contact_E = {};
contact_f = {};
task_weights = {};



for i = 1:n_dirs
    dir_path = strcat(base_dir, int2str(i));
    try
        t = load( strcat(dir_path, '/', 'time.txt') );
    catch
        fprintf('No time file found for directory number: %i\n', i)
        break;
    end
    ComTask_weights = load( strcat(dir_path, '/', 'ComTask_weights.txt') );
    FullPosture_weights = load( strcat(dir_path, '/', 'FullPosture_weights.txt') );
    LeftHandCartesian_weights = load( strcat(dir_path, '/', 'LeftHandCartesian_weights.txt') );
    RightHandCartesian_weights = load( strcat(dir_path, '/', 'RightHandCartesian_weights.txt') );
    TorsoOrientationTask_weights = load( strcat(dir_path, '/', 'TorsoOrientationTask_weights.txt') );

    ComTask_E = load( strcat(dir_path, '/', 'ComTask_E.txt') );
    ComTask_f = load( strcat(dir_path, '/', 'ComTask_f.txt') );
    FullPosture_E = load( strcat(dir_path, '/', 'FullPosture_E.txt') );
    FullPosture_f = load( strcat(dir_path, '/', 'FullPosture_f.txt') );
    LeftHandCartesian_E = load( strcat(dir_path, '/', 'LeftHandCartesian_E.txt') );
    LeftHandCartesian_f = load( strcat(dir_path, '/', 'LeftHandCartesian_f.txt') );
    RightHandCartesian_E = load( strcat(dir_path, '/', 'RightHandCartesian_E.txt') );
    RightHandCartesian_f = load( strcat(dir_path, '/', 'RightHandCartesian_f.txt') );
    TorsoOrientationTask_E = load( strcat(dir_path, '/', 'TorsoOrientationTask_E.txt') );
    TorsoOrientationTask_f = load( strcat(dir_path, '/', 'TorsoOrientationTask_f.txt') );
    
    A = load( strcat(dir_path, '/', 'One Level Solver with QuadProg++ subSolver_A.txt') );
    b = load( strcat(dir_path, '/', 'One Level Solver with QuadProg++ subSolver_b.txt') );
    G = load( strcat(dir_path, '/', 'One Level Solver with QuadProg++ subSolver_G.txt') );
    h = load( strcat(dir_path, '/', 'One Level Solver with QuadProg++ subSolver_h.txt') );
    Xsolution = load( strcat(dir_path, '/', 'One Level Solver with QuadProg++ subSolver_Xsolution.txt') );
    
    RightFootContact_BackLeft_E = load( strcat(dir_path, '/', 'RightFootContact_BackLeft_E.txt') );
    RightFootContact_BackLeft_f = load( strcat(dir_path, '/', 'RightFootContact_BackLeft_f.txt') );
    RightFootContact_BackRight_E = load( strcat(dir_path, '/', 'RightFootContact_BackRight_E.txt') );
    RightFootContact_BackRight_f = load( strcat(dir_path, '/', 'RightFootContact_BackRight_f.txt') );
    RightFootContact_FrontLeft_E = load( strcat(dir_path, '/', 'RightFootContact_FrontLeft_E.txt') );
    RightFootContact_FrontLeft_f = load( strcat(dir_path, '/', 'RightFootContact_FrontLeft_f.txt') );
    RightFootContact_FrontRight_E = load( strcat(dir_path, '/', 'RightFootContact_FrontRight_E.txt') );
    RightFootContact_FrontRight_f = load( strcat(dir_path, '/', 'RightFootContact_FrontRight_f.txt') );
    
    LeftFootContact_BackLeft_E = load( strcat(dir_path, '/', 'LeftFootContact_BackLeft_E.txt') );
    LeftFootContact_BackLeft_f = load( strcat(dir_path, '/', 'LeftFootContact_BackLeft_f.txt') );
    LeftFootContact_BackRight_E = load( strcat(dir_path, '/', 'LeftFootContact_BackRight_E.txt') );
    LeftFootContact_BackRight_f = load( strcat(dir_path, '/', 'LeftFootContact_BackRight_f.txt') );
    LeftFootContact_FrontLeft_E = load( strcat(dir_path, '/', 'LeftFootContact_FrontLeft_E.txt') );
    LeftFootContact_FrontLeft_f = load( strcat(dir_path, '/', 'LeftFootContact_FrontLeft_f.txt') );
    LeftFootContact_FrontRight_E = load( strcat(dir_path, '/', 'LeftFootContact_FrontRight_E.txt') );
    LeftFootContact_FrontRight_f = load( strcat(dir_path, '/', 'LeftFootContact_FrontRight_f.txt') );
    
    try
        endRightUpperLegContact_E = load( strcat(dir_path, '/', 'RightUpperLegContact_E.txt') );
        endRightUpperLegContact_f = load( strcat(dir_path, '/', 'RightUpperLegContact_f.txt') );

        endLeftUpperLegContact_E = load( strcat(dir_path, '/', 'LeftUpperLegContact_E.txt') );
        endLeftUpperLegContact_f = load( strcat(dir_path, '/', 'LeftUpperLegContact_f.txt') );
         
        contact_E = [contact_E; {RightFootContact_BackLeft_E, RightFootContact_BackRight_E, RightFootContact_FrontLeft_E, RightFootContact_FrontRight_E, LeftFootContact_BackLeft_E, LeftFootContact_BackRight_E, LeftFootContact_FrontLeft_E, LeftFootContact_FrontRight_E, RightUpperLegContact_E, LeftUpperLegContact_E}];
        contact_f = [contact_f; {RightFootContact_BackLeft_f, RightFootContact_BackRight_f, RightFootContact_FrontLeft_f, RightFootContact_FrontRight_f, LeftFootContact_BackLeft_f, LeftFootContact_BackRight_f, LeftFootContact_FrontLeft_f, LeftFootContact_FrontRight_f, RightUpperLegContact_f, LeftUpperLegContact_f}];

    catch
        
        contact_E = [contact_E; {RightFootContact_BackLeft_E, RightFootContact_BackRight_E, RightFootContact_FrontLeft_E, RightFootContact_FrontRight_E, LeftFootContact_BackLeft_E, LeftFootContact_BackRight_E, LeftFootContact_FrontLeft_E, LeftFootContact_FrontRight_E}];
        contact_f = [contact_f; {RightFootContact_BackLeft_f, RightFootContact_BackRight_f, RightFootContact_FrontLeft_f, RightFootContact_FrontRight_f, LeftFootContact_BackLeft_f, LeftFootContact_BackRight_f, LeftFootContact_FrontLeft_f, LeftFootContact_FrontRight_f}];
    end

    
    
    
    time = [time; t];

    task_weights = [task_weights; {ComTask_weights, FullPosture_weights, LeftHandCartesian_weights, RightHandCartesian_weights, TorsoOrientationTask_weights}];

    task_E = [task_E; {ComTask_E, FullPosture_E, LeftHandCartesian_E, RightHandCartesian_E, TorsoOrientationTask_E}];
    task_f = [task_f; {ComTask_f, FullPosture_f, LeftHandCartesian_f, RightHandCartesian_f, TorsoOrientationTask_f}];
    
    controller = [controller; {A, b, G, h, Xsolution}];
    
   
end