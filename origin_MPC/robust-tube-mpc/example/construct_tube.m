clear;
addpath('../src/')
addpath('../src/utils/')
rmpath("D:\毕设\tube_MPC\robust-tube-mpc\src\utils")
rmpath("D:\毕设\tube_MPC\robust-tube-mpc\src")
% fix random seed
rng(0);

% make your own discrete linear system with disturbance
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

W_vertex = [0.15, 0.15; 0.15, -0.15; -0.15, -0.15; -0.15, 0.15]; % construct a convex set of disturbance (2dim here)
W = Polyhedron(W_vertex);

% construct disturbance Linear system
disturbance_system = DisturbanceLinearSystem(A, B, Q, R, W);

% constraints on state Xc and input Uc
Xc_vertex = [5, -160; 5 5; -160 5; -160 -160];
Uc_vertex = [0.1,0.1;0.1,-0.5;-0.5,-0.5;-0.5,0.1];
Xc = Polyhedron(Xc_vertex);
% Xc.plot();
Uc = Polyhedron(Uc_vertex);

% create a tube_mpc simulater
% if N_horizon is too small, the path will never reach inside the robust MPI-set X_mpi_robust in time step N_horizon, then the problem becomes infeasible. 
N_horizon = 10;
w_min = [0; -0.10];
w_max = [0; 0.10];
mpc_tube = TubeModelPredictiveControl(disturbance_system, Xc, Uc, N_horizon);