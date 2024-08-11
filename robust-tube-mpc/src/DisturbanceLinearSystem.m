classdef DisturbanceLinearSystem < LinearSystem

    properties (SetAccess = private)
        W % convex set of distrubance
        w_sigma
        w_lim
        Z % disturbance invariant set
        Zk
        N
    end

    methods (Access = public)

        function obj = DisturbanceLinearSystem(A, B, Q, R, W, sigma,w_lim,N_horizon)
            
            obj = obj@LinearSystem(A, B, Q, R);
            
            obj.W = W;
            obj.w_lim =w_lim;
            obj.w_sigma = sigma;
            obj.N = N_horizon;
            obj.Z = obj.compute_mrpi_set(1e-4);
            obj.Zk = obj.compute_Zk();

        end

        function [x_new,w] = propagate(obj, x, u, w_option)
            if w_option == "random"
                w = obj.pick_random_disturbance();
            end
            if w_option == "Gaussian"
                w = obj.pick_Gaussian_disturbance();
            end
            x_new = propagate@LinearSystem(obj, x, u) + w;
        end

    end

    methods (Access = public)

        function w = pick_random_disturbance(obj)
            % pick disturbance form uniform distribution
            verts = obj.W.V;
            b_max = max(verts)';
            b_min = min(verts)';

            % generate random until it will be inside of W
            while true
                w = rand(obj.nx, 1) .* (b_max - b_min) + b_min; 
                if obj.W.contains(w)
                    break
                end
            end
        end

        function w = pick_Gaussian_disturbance(obj)
%             w = normrnd(0,obj.w_sigma);
%             [~, nw] = size(w);
%             for i=1:nw
%                 if w(1,i) >obj.w_lim
%                     w(1,i) = obj.w_lim;
%                 end
%                 if w(1,i) < -obj.w_lim
%                     w(1,i) = -obj.w_lim;
%                 end
%             end
            pdw = makedist("Normal","mu",0,"sigma",obj.w_sigma(1));
            pdw = truncate(pdw,-obj.w_lim,obj.w_lim);
            w1 = random(pdw,1,1);
            w2 = random(pdw,1,1);
            w = [w1 w2];
            w = reshape(w,2,1);
        end

        function Fs = compute_mrpi_set(obj, epsilon)
            % Computes an invariant approximation of the minimal robust positively
            % invariant set for 
            % x^{+} = Ax + w with w \in W
            % according to Algorithm 1 in 'Invariant approximations of
            % the minimal robust positively invariant set' by Rakovic et al. 
            % Requires a matrix A, a Polytope W, and a tolerance 'epsilon'.  
            [nx,~] = size(obj.Ak); 
            s = 0; 
            alpha = 1000;
            Ms = 1000;
            E = eye(nx);
            it = 0;
            tet = obj.Ak^obj.N;
            phi_W = obj.Ak^obj.N*obj.W;
%             disp(obj.K)
%             disp(obj.Ak)
%             error(" ");
            
            while((alpha > epsilon/(epsilon + Ms))&&(s<=10))
                s = s+1;
                alpha = max(phi_W.support(obj.Ak^s*(phi_W.A)')./phi_W.b);
                mss = zeros(2*nx,1);
                for i = 1:s
                    mss = mss+phi_W.support([obj.Ak^i, -obj.Ak^i]);
                end
                Ms = max(mss);
                it = it+1;
            end
            disp("finish iteration!")
%             disp(s)
            if s>=5
                s = 5;
            end
            Fs = phi_W;
            G = [1 0;0 0];
            
            for i =1:s-1
%                 Fs = Fs+obj.Ak^i*G*phi_W;
                Fs = Fs+obj.Ak^i*phi_W;
            end

            Fs = (1/(1-alpha))*Fs;
%             disp(s);
%             disp(alpha);
%             disp(obj.Ak)
        end
        function zk = compute_Zk(obj)
            zk = [];
%             phi_ = obj.A+obj.B;
            for i=1:obj.N
                if i == 1
                    zk = [0*obj.W];
                else
                    zk = [zk,zk(i-1)+obj.Ak^(i-1)*obj.W];
%                     pause(0.1)
                end
            end
        end
    end
end
