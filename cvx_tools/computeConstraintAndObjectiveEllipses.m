function [ objective_ellipsoid, constraint_ellipsoid ] = computeConstraintAndObjectiveEllipses( optima, G, h, noangles )
%COMPUTECONSTRAINTANDOBJECTIVEELLIPSES Summary of this function goes here
%   Detailed explanation goes here

if exist('noangles') == 0
    noangles = 200;
end

angles   = linspace( 0, 2 * pi, noangles );

C = optima;
%%
[m,n] = size(C);
% Create and solve the model
cvx_begin quiet
    variable A(m,m) symmetric
    variable b(m)
    maximize( det_rootn( A ) )
    subject to
        norms( A * C + b * ones( 1, n ), 2 ) <= 1;
cvx_end

if noangles ~= 0
    objective_ellipsoid.ellipse  = A \ [ cos(angles) - b(1) ; sin(angles) - b(2) ];
end
objective_ellipsoid.center = -1*A\b;
objective_ellipsoid.B = -1*inv(A);
objective_ellipsoid.d = objective_ellipsoid.center;

try
    [objective_ellipsoid.U,objective_ellipsoid.S,objective_ellipsoid.V] = svd(objective_ellipsoid.B);
catch
    objective_ellipsoid.U = zeros(m,m);
    objective_ellipsoid.S = zeros(m,m);
    objective_ellipsoid.V = zeros(m,m);
end
objective_ellipsoid.US = objective_ellipsoid.U*objective_ellipsoid.S;
objective_ellipsoid.volume = det(objective_ellipsoid.S);

objective_ellipsoid.radii = {};
for i = 1:size(objective_ellipsoid.US,2)
    point = objective_ellipsoid.center + objective_ellipsoid.US(:,i);
    radii = [objective_ellipsoid.center, point];
    objective_ellipsoid.radii = [objective_ellipsoid.radii, radii];
end

%%
[m,n] = size(G);
cvx_begin quiet
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
if noangles ~= 0
    constraint_ellipsoid.ellipse = B * [ cos(angles) ; sin(angles) ] + d * ones( 1, noangles );
end
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

