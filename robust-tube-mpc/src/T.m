classdef T < handle
    
    properties (SetAccess = public)
        sys % linear system with disturbance
        optcon; % optimal contol solver object
        Xc; % state constraint
        Uc; % input constraint
        Xc_robust; % Xc-Z (Pontryagin diff.)
        Xmpi_robust; % MPI set. Note that this is made using Xc-Z and Uc-KZ (instead of Xc and Uc)
        N; % horizon
%         nmax;%chance constraints
%         nmin;%chance constraints
        solution_cache
    end
    
    methods (Access = public)
        function obj = T(sys, X_v, Uc, N)%,nmax,nmin)
            %----------approximation of d-inv set--------%
            %create robust X and U constraints, and construct solver using X and U
            Xc_robust = [];
            for i=1:N
%                 i = N;
%                 Xk = Polyhedron(X_v(:,2*i-1:2*i));
                Xk_robust = X_v(i) - sys.Zk(i);
%                 Xk_robust.plot();
                Xc_robust = [Xc_robust,Xk_robust];
            end
%             Xc_Xmpi_robust.Vrobust = Xc - sys.Z;
%             Xc_robust.plot()
            
            Uc_robust = Uc;
%             Uc_robust.plot()
            %optcon.reconstruct_ineq_constraint(Xc_robust, Uc_robust)
%             optcon = OptimalControler(sys, Xc_robust, Uc_robust, N, nmax, nmin);
            optcon = OptimalControler(sys, Xc_robust, Uc_robust, N);
            %robustize Xmpi set and set it as a terminal constraint
%             Xmpi_robust = sys.compute_MPIset(Xc_robust(N), Uc_robust);
%             Xc_robust(1).plot()
            Xmpi_robust = sys.Ak*Xc_robust(1);
%             x_abffa = sys.Ak*Xc_robust(1);
%            sys.Z.plot()
%             Xmpi_robust = Xc_robust(N);
%             Xmpi_robust.plot();

%             optcon.add_terminal_constraint(Xmpi_robust);

            %fill properteis
            obj.sys = sys;
            obj.optcon = optcon;
            obj.Xc = X_v;
            obj.Uc = Uc;
            obj.Xc_robust = Xc_robust;
            obj.Xmpi_robust = Xmpi_robust;
            obj.N = N;
%             obj.nmax = nmax;
%             obj.nmin = nmin;
            obj.solution_cache = [];
        end

        function [u_next,Vold] = solve(obj, x_init,Vold,fmode,UC_bond)
            % Note that solution of optimal controler is cached in obj.solution_cache
%             Xinit = x_init + obj.sys.Z;
%             obj.optcon.add_initial_constraint(Xinit); %改动

            obj.optcon.add_initial_eq_constraint(x_init);
            obj.optcon.add_UC_bond_ineq_constraint(UC_bond);

            [x_nominal_seq, u_nominal_seq,exitflag] = obj.optcon.solve(Vold);
            obj.optcon.remove_constraint("UC_bond");
            if exitflag == -2
                aahdu = x_nominal_seq;
            end 
                
            obj.solution_cache = struct(...
                'x_init', x_init', 'x_nominal_seq', x_nominal_seq, 'u_nominal_seq', u_nominal_seq);

            Vold = [reshape(x_nominal_seq(:,2:end),[],1);zeros(obj.sys.nx,1);reshape(u_nominal_seq(:,2:end),[],1);zeros(obj.sys.nu,1)];
            u_nominal = u_nominal_seq(:, 1);
%             u_feedback_ = obj.sys.K * (x_init - x_nominal_seq(:, 1)); %改动
            u_feedback = obj.sys.K * (x_init); 
            if fmode == 1
                u_next = u_nominal + u_feedback;
            elseif fmode == 2
                u_next = u_feedback;
            else
                u_next = u_nominal + u_feedback;
            end
    
%             u_next = u_nominal;
        end

        function [] = show_prediction(obj)
            assert(~isempty(obj.solution_cache), 'can be used only after solved');
            Graphics.show_convex(obj.Xc, 'm');
            Graphics.show_convex(obj.Xc_robust, 'r');
            Graphics.show_convex(obj.Xmpi_robust + obj.sys.Z, [0.3, 0.3, 0.3]); % rgb = [0.3, 0.3, 0.3]
            Graphics.show_convex(obj.Xmpi_robust, [0.5, 0.5, 0.5]); % gray
            x_init = obj.solution_cache.x_init;
            scatter(x_init(1), x_init(2), 50, 'bs', 'filled');
            x_nominal_seq = obj.solution_cache.x_nominal_seq;
            Graphics.show_trajectory(x_nominal_seq, 'gs-');
            for j=1:obj.N+1
                Graphics.show_convex(x_nominal_seq(:, j)+obj.sys.Z, 'g', 'FaceAlpha', 0.1);
            end

            leg = legend({'$X_c$', '$X_c\ominus Z$', '$X_f \oplus Z$', '$X_f (X_{mpi})$', 'current state', 'nominal traj.', 'tube'}, 'position', [0.5 0.15 0.1 0.2]);
            set(leg, 'Interpreter', 'latex');
        end
    end
end
