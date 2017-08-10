



noangles = 200;

angles   = linspace( 0, 2 * pi, noangles );

a = ones(3,1);
b = ones(3,1);
c = ones(3,1);

% C = [a, b, c];
C=incomp.rollout_data.controller.metric_data{2, 2}.optima  ;
%%
[m,n] = size(C);
% Create and solve the model
% cvx_slvitr 1000;

cvx_begin sdp
    cvx_precision low
    variable A(m,m) symmetric
    variable b(m)
    maximize( det_rootn( A ) )
    subject to
        norms( A * C + b * ones( 1, n ), 2 ) <= 1;
cvx_end


center = -1*A\b;
B = -1*inv(A);
d = center;


[U,S,V] = svd(B);

US = U*S;
volume = det(S);


