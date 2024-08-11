function [MPC_Tube] = construct_Tube()
%CONSTRUCT_TUBE 此处显示有关此函数的摘要
%   此处显示详细说明
% addpath('../../../origin_MPC/robust-tube-mpc/src/')
% addpath('../../../origin_MPC/robust-tube-mpc/src/utils/')
% rmpath("D:\毕设\tube_MPC\robust-tube-mpc\src\utils")
% rmpath("D:\毕设\tube_MPC\robust-tube-mpc\src")
% % fix random seed
% rng(0);


% A = [1 1; 0 1];
% B = [0.5; 1]; 
% Q = diag([1, 1]);
% R = 0.1;
samples = 1000;
nx = 2;
nu =2;
A = [1 0; 0 1];
B = [1 0;0 1]; 
Q = [1 0; 0 1];
G = [1;0];
R = diag([1,1]);

W_vertex = [0.15, 0.15; 0.15, -0.15; -0.15, -0.15; -0.15, 0.15]; 
W = Polyhedron(W_vertex);


disturbance_system = DisturbanceLinearSystem(A, B, Q, R, W);


Xc_vertex = [5, -160; 5 5; -160 5; -160 -160];
Uc_vertex = [0.1,0.1;0.1,-0.5;-0.5,-0.5;-0.5,0.1];
Xc = Polyhedron(Xc_vertex);
% Xc.plot();
Uc = Polyhedron(Uc_vertex);


N_horizon = 10;
% w_min = [0; -0.10];
% w_max = [0; 0.10];
MPC_Tube = TubeModelPredictiveControl(disturbance_system, Xc, Uc, N_horizon);
end

