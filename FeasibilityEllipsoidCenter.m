function center = FeasibilityEllipsoidCenter( G, h )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
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

center = d;

end

