function [X_poly] = learning_constraints(Acl, G, N, xmax, xmin, epsilon,D)
%LEARNING_CONSTRAINTS 此处显示有关此函数的摘要
%   此处显示详细说明
[coffe,pd_1,pd_2] = cons_uncertainty_dis(D);
nx = size(Acl,1);
nw = size(G,2);
% pd = cell(1,nw*N);
% prin_matrix = cell(1,N);
% X_poly = cell(1,N);
W=D;
X_poly(1:N) = Polyhedron;
X_poly(1,1) = cons_uncertainty_poly(W,epsilon,xmax,xmin);
for i = 2:N
    [coffe,pd_1,pd_2,W] = propagation_pd(Acl,coffe,G,pd_1,pd_2,W);
%     pd(1,2*i-1) = pd_1;
%     pd(1,2*i) = pd_2;
%     prin_matrix(1,i) = coffe;
    [X_i] = cons_uncertainty_poly(W,epsilon,xmax,xmin);
    X_poly(1,i) = X_i;
end
% outputArg1 = inputArg1;
% outputArg2 = inputArg2;
end

