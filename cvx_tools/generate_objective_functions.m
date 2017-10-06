clear all;
clc;

m = 2; n = 2;

A1 = randn(m,n);
b1 = randn(m,1);

A2 = randn(m,n);
b2 = randn(m,1);

A3 = randn(m,n);
b3 = randn(m,1);

A4 = randn(m,n);
b4 = randn(m,1);

A5 = randn(m,n);
b5 = randn(m,1);

A6 = randn(m,n);
b6 = randn(m,1);

save('A_mats_01.mat', 'A1', 'A2', 'A3', 'A4', 'A5', 'A6')
save('b_vecs_01.mat', 'b1', 'b2', 'b3', 'b4', 'b5', 'b6')