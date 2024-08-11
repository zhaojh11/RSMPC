clear;
addpath('../src/')
addpath('../src/utils/')
% rmpath("")

rng(0);
A = [0.00 0.00 0.0 0.0; 1 0 0 0;0 0 0 0;0 0 1 0];
B = [1 0;0 0; 0 1; 0 0]; 
Q = [1 1 1 1; 1 1 1 1;1 1 1 1; 1 1 1 1];
G = [1 0;0 0;0 1;0 0];
R = 1;

epsilon = [0.1;NaN];
var = blkdiag(0, 0);
sigma = 0.08;
Qw = sigma^2;
N_horizon = 8;

W_vertex = [0.15, 0.15; 0.15, -0.15; -0.15, -0.15; -0.15, 0.15];
W = Polyhedron(W_vertex);
w_lim =0.7;
% construct disturbance Linear system
flag = 0;
A = [0.001 0.001; 1 0];
disturbance_system = DisturbanceLinearSystem(A, B, Q, R, W, [sigma 0],w_lim,N_horizon,flag);