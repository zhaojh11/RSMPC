function [MPC_SMPC] = construct_SMPC()
%CONSTRUCT_SMPC 此处显示有关此函数的摘要
%   此处显示详细说明
    % addpath('../')
    % addpath('../../src/')
    % addpath('../../src/utils/')
    % rmpath("D:\毕设\tube_MPC\origin_MPC\robust-tube-mpc\src\utils")
    % rmpath("D:\毕设\tube_MPC\origin_MPC\robust-tube-mpc\src")
    % rng(0)
    % 不确定数据生成
    
    mu = [0,0];
    SIGMA = [0.07^2,-0.003;-0.003,0.07^2];
    D = mvnrnd(mu,SIGMA,1000);
    
    % control_body
    
    A = [1 0;0 1];
    B = [1 0;0 1]; 
    Q = [1 0; 0 1]; 
    G = [1 0;0 1];
    R = diag([1,1]);
    
    epsilon = [0.05;0.05];
    sigma = 0.07;
    N_horizon = 10;
    % Vold = zeros(size(B,1)*(N_horizon+1)+size(B,2)*N_horizon,1);
    W_vertex = [0.15, 0.15; 0.15, -0.15; -0.15, -0.15; -0.15, 0.15]; % construct a convex set of disturbance (2dim here)
    W = Polyhedron(W_vertex);
    w_lim =0.7;

    disturbance_system = DisturbanceLinearSystem(A, B, Q, R, W, [sigma sigma],w_lim,N_horizon);

    Uc_vertex1 = [0.01,0.01;0.01,-0.2;-0.2,-0.2;-0.2,0.01];
    Uc_before = Polyhedron(Uc_vertex1);
    % Uc_vertex2 = [0.2,0.2;0.2,-0.05;-0.05,-0.05;-0.05,0.2];
    % Uc_after = Polyhedron(Uc_vertex2);
    x_min = [-60;-60];
    x_max = [1.2;1.2];
    % [nx,~] = size(B);
    X_poly_learning = learning_constraints(disturbance_system.Ak,G,N_horizon,x_max,x_min,epsilon+0.05,D); 
    

    MPC_SMPC = T(disturbance_system, X_poly_learning, Uc_before, N_horizon);
    
    % [C_eq1, C_eq2] = mpc_smpc.optcon.constraint_manager.combine_all_eq_constraints();
    % [C_ineq1, C_ineq2] = mpc_smpc.optcon.constraint_manager.combine_all_ineq_constraints();
    % H = mpc_smpc.optcon.H;
    
    % mpc_after = TubeModelPredictiveControl(disturbance_system, X_poly_learning, Uc_after, N_horizon);

end

