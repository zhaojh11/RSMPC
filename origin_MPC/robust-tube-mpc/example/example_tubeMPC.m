clear 

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
Xc_vertex = [0.15, -160; 0.15 0.15; -160 0.15; -160 -160];
Uc_vertex = [0.1,0.1;0.1,-0.5;-0.5,-0.5;-0.5,0.1];
Xc = Polyhedron(Xc_vertex);
% Xc.plot();
Uc = Polyhedron(Uc_vertex);

% create a tube_mpc simulater
% if N_horizon is too small, the path will never reach inside the robust MPI-set X_mpi_robust in time step N_horizon, then the problem becomes infeasible. 
N_horizon = 10;
w_min = [0; -0.10];
w_max = [0; 0.10];
mpc = TubeModelPredictiveControl(disturbance_system, Xc, Uc, N_horizon);

% The robust MPC guidances the path inside the robust MPI-set so that the path will reach the robust MPI-set in N_horizon. 
x = [-7; -2];
savedir_name = './results/';
mkdir(savedir_name);
% 
realization=1;
xrealv = zeros(realization,samples+1,2);
urealv = zeros(realization,samples+1,2);
% realization=100;
for dawd = 1:realization
    x = [-55;-55];
    xrealv(dawd,1,:) = x;
for i = 2:samples
%     disp(i)
    u_next = mpc.solve(x);
    x = disturbance_system.propagate(x, u_next); % additive disturbance is considered inside the method 
    xrealv(dawd,i,:) = x';
    urealv(dawd,i,:)=u_next';
%     mpc.show_prediction();
%     filename = strcat(savedir_name, 'tmpc_seq', number2string(i), '.png')
%     saveas(gcf, char(filename)); % removing this line makes the code much faster
%     clf;
end
figure(1)
for j = 1:2
    subplot(nx,1,j)
    line([0,17],[0.15,0.15],'Color','r','linestyle','--')
    if flag == 0
        h1 = plot(1: samples+1, xrealv(dawd,:,j),"Color",[0.4 0.7 1.0]);
%                 legend([h1,h2],"real",'true')
    else
        h2 = plot(1: samples+1, xrealv(dawd,:,j),"Color",[1.0 0.7 0.1]);
%                 legend(h2,"real")
    end
    if j==1
        title('real states evolution')
    end
    hold on
    ylabel(['x', num2str(j)])
    if j ~= nx
        xticklabels({})
    end
end
end
% save("tube_date.mat",'xrealv')
save('tube_u.mat','urealv')
