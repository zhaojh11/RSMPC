clear;
addpath('../src/')
addpath('../src/utils/')

% fix random seed
rng(0);

% make your own discrete linear system with disturbance
A = [0 0; 1 0];
B = [1; 0]; 
Q = [1 1; 1 1];
R = 1;

W_vertex = [0.15, 0.0; 0.15, -0.0; -0.15, -0.0; -0.15, 0.0]; % construct a convex set of disturbance (2dim here)
W = Polyhedron(W_vertex);
% W.plot();
% error(" ");
% construct disturbance Linear system
disturbance_system = DisturbanceLinearSystem(A, B, Q, R, W);
% z = disturbance_system.Z;
% z.plot();
% error(" ");

% constraints on state Xc and input Uc
Xc_vertex = [10, 3; 10 -10; -10 -10; -10 3];
Uc_vertex = [1; -1];
Xc = Polyhedron(Xc_vertex);
Uc = Polyhedron(Uc_vertex);

% create a tube_mpc simulater
% if N_horizon is too small, the path will never reach inside the robust MPI-set X_mpi_robust in time step N_horizon, then the problem becomes infeasible. 
N_horizon = 10;
% w_min = [0; -0.10];
% w_max = [0; 0.10];
mpc = TubeModelPredictiveControl(disturbance_system, Xc, Uc, N_horizon);

% The robust MPC guidances the path inside the robust MPI-set so that the path will reach the robust MPI-set in N_horizon. 
% x = [-3.5; -3.5];
savedir_name = './results/';
mkdir(savedir_name);
% 设定值
samples = 100;
nx =2;
nu =1;
xrealv = zeros(samples+1,2);
wrealv = zeros(samples+1,2);
urealv = zeros(samples+1,1);
realization = 2;

% xrealv(1,:) = x;
urealv(1,:) = 0;
wrealv(1,:) = [0;0];

%

for dawd = 1:realization
    x = [-7;-7];
    xrealv(1,:) = x;
    for i = 2:samples
        u_next = mpc.solve(x);
%         u_next = 0;
        [x,w] = disturbance_system.propagate(x, u_next); % additive disturbance is considered inside the method 
        
        wrealv(i,:) = w';
        xrealv(i,:) = x';
        urealv(i,:) = u_next';
    
%         mpc.show_prediction();
%         pause(0.1)
%         filename = strcat(savedir_name, 'tmpc_seq', number2string(i), '.png');
%         saveas(gcf, char(filename)); % removing this line makes the code much faster
%         clf;
    end
    
    figure(1)
    for j = 1:nx
        subplot(nx,1,j)
        plot(1: samples+1, xrealv(:,j),"Color",[0.1 0.7 1]);
        if j==1
            title('real states evolution')
        end
        hold on
        ylabel(['x', num2str(j)])
        if j ~= nx
            xticklabels({})
        end
    end
    xlabel('samples (k)')
    
    % graphs of the inputs:
    figure(2)
    for j = 1:nu
        subplot(nu,1,j)
        plot(0:samples, urealv(:,j),"Color",[0.1 0.7 1]);
        if j==1
            title('inputs evolution')
        end
        hold on
        ylabel(['u', num2str(j)])
        if j ~= nu
            xticklabels({})
        end
    end
    xlabel('samples (k)')

    figure(3)
    for j = 1:nx
        subplot(nx,1,j)
        plot(1: samples+1, wrealv(:,j),"Color",[0.1 0.7 1]);
        if j==1
            title('real disturbance evolution')
        end
        hold on
        ylabel(['w', num2str(j)])
        if j ~= nx
            xticklabels({})
        end
    end
    xlabel('samples (k)')
end