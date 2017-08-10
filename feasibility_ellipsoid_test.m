G = infeas.rollout_data.controller.G;
h = infeas.rollout_data.controller.h;

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