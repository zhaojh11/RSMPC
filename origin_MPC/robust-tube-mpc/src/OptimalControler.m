classdef OptimalControler < handle
    % Let s :=[x(0), .., x(n), u(0)....u(n-1)] be decision variables, then
    % optimal control problem is composed by
    % (i) objective function V(s) = s'*H*S in quadratic form
    % (ii) equality constraints specified by C_eq1 and C_eq2 where C_eq1*s =C_eq2
    % (iii) inequality constraints specified by C_ineq1 and C_ineq2 where C_ineq1 * s <= C_ineq2
    %
    % For potential future extensions, all constraints will be managed by ConstraintManager class.
    % Any time you add a new constraint, those constraint is pushed into the manager with a key (name of constraint). 
    
    
    properties (SetAccess = private)
        sys; %system
        Xc; Uc; % constraints set for statespace and input space
        x_min; x_max; % lower and upper bound of Xc
        N; % prediction horizon
        Ak; % S.T.M of closed-roop system with LQR feedback
        n_opt; % dim. of optimization parameter s :=[x(0), .., x(n), u(0)....u(n-1)]
        H; % positive definite matrix for objective function V(s) = s'*H*s 
        constraint_manager;
        pre_optim;
    end
    
    %% Public Methods
    methods (Access = public)
        
        function obj = OptimalControler(sys, Xc, Uc, N)
            obj.sys = sys;
            obj.Xc = Xc;
            obj.x_min = min(Xc.V, [], 1)';
            obj.x_max = max(Xc.V, [], 2)';
            obj.Uc = Uc;
            obj.N = N;
            
            obj.n_opt = obj.sys.nx*(obj.N+1)+obj.sys.nu*obj.N;
            obj.constraint_manager = ConstraintManager();
            
            obj.H = obj.construct_costfunction();
            [C_eq1, C_eq2] = obj.construct_dynamics_constraint();
            [C_ineq1, C_ineq2] = obj.construct_ineq_constraint(Xc, Uc);

            %% Let's change initial and dynamics constraints!!
            obj.constraint_manager.add_eq_constraint('dynamics', C_eq1, C_eq2);
            obj.constraint_manager.add_ineq_constraint('feasible', C_ineq1, C_ineq2);
        end

        function add_initial_eq_constraint(obj, x_init)
            % E * x0 = x_init
            idx_x0_start = 1;
            idx_x0_end = obj.sys.nx;

            C_eq1_init = zeros(obj.sys.nx, obj.n_opt);
            C_eq1_init(:, idx_x0_start:idx_x0_end) = eye(obj.sys.nx);
            C_eq2_init = x_init;
            obj.constraint_manager.add_eq_constraint('initial', C_eq1_init, C_eq2_init);
        end
        function remove_constraint(obj,key)
            obj.constraint_manager.remove_ineq_constraint(key);
        end

        function add_terminal_constraint(obj, Xadd)
            [C_ineq1_add, C_ineq2_add] = add_ineq_constraint(obj, Xadd, obj.N+1);
            obj.constraint_manager.add_ineq_constraint('terinal', C_ineq1_add, C_ineq2_add);
        end

        function add_initial_constraint(obj, Xadd)
            [C_ineq1_add, C_ineq2_add] = add_ineq_constraint(obj, Xadd, 1);
            obj.constraint_manager.add_ineq_constraint('initial', C_ineq1_add, C_ineq2_add);
        end
     
        function add_UC_bond_ineq_constraint(obj, UC_bond)
            C_ineq1 = [];
            C_ineq2 = [];
            min_v = UC_bond(1,2);
            max_v = UC_bond(1,1);
            
            G_add = [1 0;-1 0;0 1;0 -1];
            C_ineq2_add = [max_v;-min_v;max_v;-min_v];
            G_block = zeros(size(G_add,1),obj.sys.nx*(obj.N+1));
            for i=1:obj.N
                G_U_block = [G_block,zeros(size(G_add,1),obj.sys.nu*(i-1)),G_add,zeros(size(G_add,1),obj.sys.nu*(obj.N-i))];
                C_ineq1 = [C_ineq1;G_U_block];
                C_ineq2 = [C_ineq2;C_ineq2_add];
            end
            obj.constraint_manager.add_ineq_constraint('UC_bond',C_ineq1,C_ineq2);
        end
        
        function [x_seq, u_seq,obj] = solve(obj)
            quadprog_solved = 0;
            [C_eq1, C_eq2] = obj.constraint_manager.combine_all_eq_constraints();
            [C_ineq1, C_ineq2] = obj.constraint_manager.combine_all_ineq_constraints();

            options = optimoptions('quadprog', 'Display', 'none');
            [var_optim, ~, exitflag] = quadprog(obj.H, [], C_ineq1, C_ineq2, C_eq1, C_eq2, [], [], [], options);
            if exitflag == -2
                var_optim = obj.pre_optim;
            end
            obj.pre_optim = var_optim;
            x_seq = reshape(var_optim(1:obj.sys.nx*(obj.N+1)), obj.sys.nx, obj.N+1);
            u_seq = reshape(var_optim(obj.sys.nx*(obj.N+1)+1:obj.n_opt), obj.sys.nu, obj.N);
            
        end
        
    end
    
    %% Methods Used in Constoructor
    methods (Access = private)
        
        function H = construct_costfunction(obj)
            % compute H
            Q_block = [];
            R_block = [];
            for itr=1:obj.N
                Q_block = blkdiag(Q_block, obj.sys.Q);
                R_block = blkdiag(R_block, obj.sys.R);
            end
            H = blkdiag(Q_block, obj.sys.P, R_block);
        end

        function [C_eq1, C_eq2] = construct_dynamics_constraint(obj)
            % compute C_eq1 and C_eq2
            function C_ss_eq1 = single_step_dynamics_eq1(k)
                % A x(k) - E x(k+1) + Bu(k) = 0
                idx_xk_start = obj.sys.nx * k + 1;
                idx_xk_end = obj.sys.nx * (k + 1);

                idx_xkp1_start = obj.sys.nx * (k + 1) + 1;
                idx_xkp1_end = obj.sys.nx * (k + 2);

                idx_uk_start  = obj.sys.nx * (obj.N+1) + obj.sys.nu * k + 1;
                idx_uk_end  = obj.sys.nx * (obj.N+1) + obj.sys.nu * (k + 1);

                C_ss_eq1 = zeros(obj.sys.nx, obj.n_opt);

                C_ss_eq1(:, idx_xk_start:idx_xk_end) = obj.sys.Ak;
                C_ss_eq1(:, idx_xkp1_start:idx_xkp1_end) = - eye(obj.sys.nx);
                C_ss_eq1(:, idx_uk_start:idx_uk_end) = obj.sys.B;
            end

            C_eq1 = [];
            for k = 0:obj.N-1
                C_ss_eq1 = single_step_dynamics_eq1(k);
                C_eq1 = [C_eq1; C_ss_eq1];
            end
            C_eq2 = zeros(size(C_eq1, 1), 1);
        end
       
        function [C_ineq1, C_ineq2] = construct_ineq_constraint(obj, Xc, Uc)
            % compute C_ineq
%             if Xc.contains(zeros(2, 1)) % If Xadd contains the origin, the contraint can be expressed as C1*x<=1
%                 [F, ~, nc_add] = convert_Poly2Mat(Xc, Polyhedron());
%                 C_ineq2_add = ones(nc_add, 1);
%                 
%             else % in other cases, expressed in a general affine form C1*x<=C2
%                 F = Xc.A;
%                 nc_add = size(F, 1);
%                 C_f_add = Xc.b;
%             end  
% 
%             G = Uc.A;
%             nc_add = size(G, 1);
%             C_g_add = Uc.b;
% %             end  
%             [F, G, nc] = convert_Poly2Mat(Xc, Uc);
%             
%             F_block = [];
%             G_block = [];
%             for itr = 1:obj.N
%                 G_block = blkdiag(G_block, G);
%             end
%             for itr = 1:obj.N+1
%                 F_block = blkdiag(F_block, F);
%             end
%             C_ineq1 = [F_block, [G_block; zeros(nc, obj.sys.nu*obj.N)]];
%             nc_total = size(C_ineq1, 1);
%             C_ineq2 = ones(nc_total, 1);
            C_ineq1 = [];
            C_ineq2 = [];
            for i=1:obj.N
                Xkc = Xc;
%                 Xkc.plot();
%                 Uc.plot();
%                 [F, G, ~] = convert_Poly2Mat(Xkc, Uc);
                if Xkc.contains(zeros(2, 1)) % If Xadd contains the origin, the contraint can be expressed as C1*x<=1
                    [F_add, ~, nfc_add] = convert_Poly2Mat(Xkc, Polyhedron());
                    C_ineq2_add = ones(nfc_add, 1);
                    
                else % in other cases, expressed in a general affine form C1*x<=C2
                    F_add = Xkc.A;
                    nfc_add = size(F_add, 1);
                    C_ineq2_add = Xkc.b;
                end
                G_block = zeros(size(F_add,1),obj.sys.nu*obj.N);
                F_block = [zeros(size(F_add,1),obj.sys.nx*(i-1)),F_add,zeros(size(F_add,1),obj.sys.nx*(obj.N-i+1))];
                C_ineq1 = [C_ineq1 ; [F_block, G_block]];
                C_ineq2 = [C_ineq2;C_ineq2_add];
            end
            % if Uc.contains(zeros(2, 1)) % If Xadd contains the origin, the contraint can be expressed as C1*x<=1
            %     [G_add, ~, ngc_add] = convert_Poly2Mat(Uc, Polyhedron());
            %     C_ineq2_add = ones(ngc_add, 1);
            % 
            % else % in other cases, expressed in a general affine form C1*x<=C2
            %     G_add = Uc.A;
            %     ngc_add = size(G_add, 1);
            %     C_ineq2_add = Uc.b;
            % end  
            % 
            % G_block = zeros(size(G_add,1),obj.sys.nx*(obj.N+1));
            % for i=1:obj.N
            %     G_U_block = [G_block,zeros(size(G_add,1),obj.sys.nu*(i-1)),G_add,zeros(size(G_add,1),obj.sys.nu*(obj.N-i))];
            %     C_ineq1 = [C_ineq1;G_U_block];
            %     C_ineq2 = [C_ineq2;C_ineq2_add];
            % end
        end
        
    end
    
    methods (Access = private)
        
        function [C_ineq1_add, C_ineq2_add] = add_ineq_constraint(obj, Xadd, k_add)
            % add a new constraint at time step k 
            if Xadd.contains(zeros(2, 1)) % If Xadd contains the origin, the contraint can be expressed as C1*x<=1
                [F_add, ~, nc_add] = convert_Poly2Mat(Xadd, Polyhedron());
                C_ineq2_add = ones(nc_add, 1);
                
            else % in other cases, expressed in a general affine form C1*x<=C2
                F_add = Xadd.A;
                nc_add = size(F_add, 1);
                C_ineq2_add = Xadd.b;
            end
            
            C_ineq1_add = zeros(nc_add, obj.n_opt);
            C_ineq1_add(:, (k_add-1)*obj.sys.nx+1:k_add*obj.sys.nx) = F_add;
        end
        
    end
end

