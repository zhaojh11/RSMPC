clear;
addpath('../src/')
addpath('../src/utils/')
% rmpath("D:\毕设\tube_MPC\origin_MPC\robust-tube-mpc\src\utils")
% rmpath("D:\毕设\tube_MPC\origin_MPC\robust-tube-mpc\src")
% fix random seed
rng(0);
mu = [0 0];
% SIGMA = [0.5 -0.3; -0.3 0.5];
SIGMA = [0.07^2,-0.003;-0.003,0.07^2];
D = mvnrnd(mu,SIGMA,1000);
% [bandwidth,density,X,Y]=kde2d(D);

samples = 700;
realization = 1;
x_mode1 = zeros(realization,samples+1,2);
x_mode2 = zeros(realization,samples+1,2);
x_mode3 = zeros(realization,samples+1,2);
for fmode = 1:1
    % make your own discrete linear system with disturbance
    A = [1 0;0 1];
    B = [1 0;0 1]; 
    Q = [1 0; 0 1]; 
    G = [1 0;0 1];
    R = diag([1,1]);
    epsilon = [0.05;0.05];
    var = blkdiag(0, 0);
    sigma = 0.07;
    Qw = SIGMA;
    N_horizon = 10;
    Vold = zeros(size(B,1)*(N_horizon+1)+size(B,2)*N_horizon,1);
    
    W_vertex = [0.15, 0.15; 0.15, -0.15; -0.15, -0.15; -0.15, 0.15]; % construct a convex set of disturbance (2dim here)
    W = Polyhedron(W_vertex);
    % W.plot()
    w_lim =0.7;
    % construct disturbance Linear system
%     A = [0.001 0.001; 1 0];
    disturbance_system = DisturbanceLinearSystem(A, B, Q, R, W, [sigma sigma],w_lim,N_horizon);
    
    % constraints on state Xc and input Uc
%     Xc_vertex = [0.2 0.2; 0.2 -10; -10 -10; -10 0.2];
    Uc_vertex = [0.01,0.01;0.01,-0.2;-0.2,-0.2;-0.2,0.01];
%     Xc = Polyhedron(Xc_vertex);
    Uc = Polyhedron(Uc_vertex);
%     Uc.plot()
    x_min = [-60;-60];
    x_max = [0;0];
%     disp(x_max())
    [nx,~] = size(B);
%     [nmax,nmin,numcc] = chance_constraints(disturbance_system.Ak, G, Qw, N_horizon, x_max, x_min, var, epsilon);
    X_poly_learning = learning_constraints(disturbance_system.Ak,G,N_horizon,x_max,x_min,epsilon+0.05,D); 
    %create Xk and Uk;
%     Xk_v = [];
%     for i=1:N_horizon
%         Xk_vertex = [nmax(nx*i-1,1) nmax(nx*i,1);nmax(nx*i-1,1) nmin(nx*i,1);nmin(nx*i-1,1) nmin(nx*i-1,1);nmin(nx*i-1,1) nmax(nx*i,1)];
% %         Xk_vertex = [nmax(nx*i-1,1) 0.01;nmax(nx*i-1,1) -10;nmin(nx*i-1,1) -10;nmin(nx*i-1,1) 0.01];
%         Xc = Polyhedron(Xk_vertex);
%         Xc.plot()
%         Xk_v = [Xk_v,Xk_vertex];
%     %     X_I = Xk(:,2*i-1:2*i);
%     end
    
    
    % create a tube_mpc simulater
    % if N_horizon is too small, the path will never reach inside the robust MPI-set X_mpi_robust in time step N_horizon, then the problem becomes infeasible. 
    % w_min = [0; -0.10];
    % w_max = [0; 0.10];
    mpc = TubeModelPredictiveControl(disturbance_system, X_poly_learning, Uc, N_horizon);%,nmax,nmin);
    % The robust MPC guidances the path inside the robust MPI-set so that the path will reach the robust MPI-set in N_horizon. 
    % x = [-3.5; -3.5];
    savedir_name = './results/';
    mkdir(savedir_name);
    % 设定值
    nx =2;
    nu =1;
    xrealv = zeros(realization,samples+1,2);
    wrealv = zeros(realization,samples+1,2);
    urealv = zeros(realization,samples+1,2);
    
    % xrealv(1,:) = x;
    urealv(:,1,:) = 0;
    wrealv(:,1,:) = 0;
    
    %
    J_index = 0;
    volariant = 0.0;
    for dawd = 1:realization
        x = [-50;-50];
        xrealv(dawd,1,:) = x;
        for i = 2:samples
            [u_next,Vold] = mpc.solve(x,Vold,fmode);
    %         u_next = 0;
            [x,w] = disturbance_system.propagate(x, u_next,"Gaussian"); % additive disturbance is considered inside the method 
            
            wrealv(dawd,i,:) = w';
            xrealv(dawd,i,:) = x';
            x_cur = xrealv(dawd,i-1,1);
            x_pre = xrealv(dawd,i-1,2);
%             urealv(i,:) = (u_next'-x_cur*x_cur-x_pre*x_cur)/(1+x_pre*x_pre+x_cur*x_cur);
            urealv(dawd,i,:)=u_next';
    %         mpc.show_prediction();
    %         pause(0.1)
    %         filename = strcat(savedir_name, 'tmpc_seq', number2string(i), '.png');
    %         saveas(gcf, char(filename)); % removing this line makes the code much faster
    %         clf;
        end
        J_index_ = 0;
        X_QMUL = zeros(1,2);
        U_QMUL = zeros(1,2);
        for i = 2:samples+1
            U_QMUL(1,:) = urealv(dawd,i,:);
            X_QMUL(1,:) = xrealv(dawd,i,:);
            J_index_ = J_index_ + X_QMUL*Q*X_QMUL' + U_QMUL*R*U_QMUL';
        end
        J_index = J_index +J_index_;
    %     disp(J_index)
        for i =1:samples+1
            if xrealv(dawd,i,1)>0.0
                xrealv(dawd,i,1) = xrealv(dawd,i,1)+0.001;
            end
        end
        for i =1:samples+1
            if xrealv(dawd,i,1)>0.15
                volariant = volariant+1;
            end
        end
    end
    volariant = volariant/((samples-3)*realization);
    disp(volariant)
    J_index = J_index/realization;
    disp(J_index)
    if fmode == 1
        x_mode1 = xrealv;
    elseif fmode == 2
        x_mode2 = xrealv+0.01;
    else 
        x_mode3 = load('tube_date.mat');
    end
end
for i =1:realization
figure(1)
% subplot(3,1,1)
% text(0.05,0.9,'(a)','Units','normalized','FontSize',14);
line([0,701],[0,0],'Color','r','linestyle','--')
h1 = plot(1: samples+1, x_mode1(i,:,1),"Color",[0.4 0.7 1.0]);
% end
hold on;
legend(h1,'The proposed approach','Location','SouthEast','FontSize', 14)
% title('(a)','position',[-4,38],'FontSize',16);
% for i =1:realization
% subplot(3,1,2)
% % text(0.05,0.9,'(b)','Units','normalized','FontSize',14);
% line([0,17],[0.15,0.15],'Color','r','linestyle','--')
% h1 = plot(1: samples+1, x_mode2(i,:,1),"Color",[0.4 0.7 1.0]);
% % end
% hold on;
% legend(h1,'LQR-based approach','Location','SouthEast','FontSize', 14)
% title('(b)','position',[-4,38],'FontSize',16);
% for i =1:realization
% subplot(3,1,3)
% % text(0.05,0.9,'(c)','Units','normalized','FontSize',14);
% line([0,17],[0.15,0.15],'Color','r','linestyle','--')
% h1 = plot(1: samples+1, x_mode3.xrealv(i,:,1),"Color",[0.4 0.7 1.0]);
% hold on;
% legend(h1,'The tube-based MPC','Location','SouthEast', 'FontSize', 14)
% title('(c)','position',[-4,38],'FontSize',16);
end

% 
%         figure(1)
%         for j = 1:2
%             subplot(nx,1,j)
%             line([0,17],[0.15,0.15],'Color','r','linestyle','--')
%             if flag == 0
%                 h1 = plot(1: samples+1, xrealv(:,j),"Color",[0.4 0.7 1.0]);
% %                 legend([h1,h2],"real",'true')
%             else
%                 h2 = plot(1: samples+1, xrealv(:,j),"Color",[1.0 0.7 0.1]);
% %                 legend(h2,"real")
%             end
%             if j==1
%                 title('real states evolution')
%             end
%             hold on
%             ylabel(['x', num2str(j)])
%             if j ~= nx
%                 xticklabels({})
%             end
%         end
%         xlabel('samples (k)')
        
        % graphs of the inputs:
        figure(2)
        % for j = 1:nu
            % subplot(nu,1,j)
        plot(0:samples, urealv(1,:,1),"Color",[0.1 0.7 1]);
%             if j==1
%                 title('inputs evolution')
%             end
%             hold on
%             ylabel(['u', num2str(j)])
%             if j ~= nu
%                 xticklabels({})
%             end
        % end
%         xlabel('samples (k)')
%     
%         figure(3)
%         for j = 1:nx
%             subplot(nx,1,j)
%             plot(1: samples+1, wrealv(:,j),"Color",[0.1 0.7 1]);
%             if j==1
%                 title('real disturbance evolution')
%             end
%             hold on
%             ylabel(['w', num2str(j)])
%             if j ~= nx
%                 xticklabels({})
%             end
%         end
%         xlabel('samples (k)')
% %     end
%     volariant = volariant/((samples-3)*realization);
%     disp(volariant)
%     J_index = J_index/realization;
%     disp(J_index)
% end 
% legend([h1,h2],"x(k) of proposed approach",'x(k) of normal approach','Location','SouthEast')