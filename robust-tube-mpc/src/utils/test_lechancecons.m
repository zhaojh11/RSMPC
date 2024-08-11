addpath("../../example/")

clear;
rng(0);
mu = [0 0];
% SIGMA = [0.5 -0.3; -0.3 0.5];
SIGMA = [0.1^2,0.003;0.003,0.1^2];
D = mvnrnd(mu,SIGMA,1000);

A = [0.21 0.308;0.308 -0.0894];
B = [1 0;0 1]; 
Q = [1 0; 0 1];
G = [1 0;0 1];
R = diag([1,1]);
epsilon = [0.05;0.05];
var = blkdiag(0, 0);
sigma = 0.1;
Qw = [0.06^2,-0.005;-0.005,0.06^2];
N_horizon = 5;
% norminv(0.95,0,0.1)

x_min = [-10;-10];
x_max = [0.15;0.15];
%     disp(x_max())
[nx,~] = size(B);
% [nmax,nmin,numcc] = chance_constraints(disturbance_system.Ak, G, Qw, N_horizon, x_max, x_min, var, epsilon,1);
X_poly = learning_constraints(A,G,N_horizon,x_max,x_min,epsilon,D);
figure
for i = 1:N_horizon
    subplot(N_horizon,1,i)
    X_poly(i).plot();
end