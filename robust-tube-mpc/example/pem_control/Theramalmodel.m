classdef Theramalmodel < handle
    %THERAMALMODEL 此处显示有关此类的摘要
    %电解槽的热力学建模
    
    properties
        %parameters
        C_t = 42045;
        C_pc = 4180;
        A_hxH_hx = 380;
        R_t = 0.082;
        T_0 = 25;
        T_a = 25;
        T_ci = 20;
        N = 100;
        I = 170;
        U_th = 1.48;
        U = 2.0;
        delta = 0.25;
        delta_mw;
        
        tao_t = 0.082*42045;
        % m_c = 0.25; 
        C_c; 
        
        T_initial;
        current_T;
        fail;
        pre_wf;
    end
    
    methods
        function obj = Theramalmodel(T_initial)
            %THERAMALMODEL 构造此类的实例
            %
            obj.T_initial = T_initial;
            obj.current_T = T_initial;
            obj.fail = 0;
            obj.pre_wf = 0;
            obj.delta_mw = 0.2*obj.delta;
        end
        function x = input(obj,current_density)
            %电解槽的电压建模
            R = 8.314;
            alpha_a = 0.5;
            alpha_c = 0.5;
            z = 2;
            F = 96485.33289;
            % i_c = 0;
            i_o_c = 1E-3;
            % i_a = 0;
            i_o_a = 3E-6;
            R_ele = 3.5E-5;
            t_mem = 1.27E-2;
            lamda = 12.7;
            
            Area = 170;
            NC = 100;
            V_th = 1.48;
            i = current_density;
            % C_p = 42045;
            temp = obj.current_T+273;

            V_rev = 1.5184 - 1.5421E-3*temp+9.523E-5*temp*log(temp)+9.84E-8*temp^2;
            R_mem = t_mem/((0.005139*lamda - 0.00326)*exp(1268*(1/303-1/temp)));
            v_cell = (V_rev + (R*temp/(alpha_a*z*F))*log(i/i_o_a) + (R*temp/(alpha_c*z*F))*log(i/i_o_c) + (R_ele+R_mem)*i-V_th);
            x =v_cell*i*Area*NC*sqrt(25/obj.current_T);
           
        end

        function [x,obj] = next_Temp(obj,current_density,m_w)
            %下一个采样时刻的电解槽温度
            obj.C_c = m_w*obj.C_pc;
            alpha = 1/obj.tao_t + (obj.C_c/obj.C_t)*(1-exp(-obj.A_hxH_hx/obj.C_c));
            belta = obj.input(current_density)/obj.C_t + obj.T_a/obj.tao_t + (obj.C_c*obj.T_ci/obj.C_t)*(1-exp(-obj.A_hxH_hx/obj.C_c));
            % x = (-alpha*obj.current_T+belta)*obj.delta + obj.current_T;
            % afah = (-alpha*obj.current_T+belta)*obj.delta;
            obj.current_T = (-alpha*obj.current_T+belta)*obj.delta + obj.current_T+obj.disturbance;
            x = obj.current_T;
        end
        
        function x = disturbance(obj)
            %电解槽中基于采样时刻的噪声
            x = 0.07*randn(1,1)*sqrt(obj.delta);
        end
        function [output,obj] = solve_wf(obj,current_density,fmode,set_point)
            %求解当前温度的合适的水流
            %fmode：应用哪种控制方法：1表示Tube-based MPC；2表示提出的方法；3表示LQR
            
            function y = solve_water_flow(obj,current_density,constant,m_w)
                % C_pc = 4180;
                C_c_s = m_w*obj.C_pc;
                alpha = 1/obj.tao_t + (C_c_s/obj.C_t)*(1-exp(-obj.A_hxH_hx/C_c_s));
                belta = obj.input(current_density)/obj.C_t + obj.T_a/obj.tao_t + (C_c_s*obj.T_ci/obj.C_t)*(1-exp(-obj.A_hxH_hx/C_c_s));
                % aawinio = (-1/obj.tao_t)*obj.current_T+obj.input(current_density)/obj.C_t + obj.T_a/obj.tao_t;
                y = (-alpha*obj.current_T+belta)*obj.delta-constant;
            end

            u_feedback = -0.01*(obj.current_T-set_point)*obj.delta;
            bond = @(z)solve_water_flow(obj,current_density,0,z);
            if (obj.pre_wf-obj.delta_mw) <= 0 
                min_v = 0 ;
            else 
                min_v = obj.pre_wf-obj.delta_mw ;
            end
            if (obj.pre_wf+obj.delta_mw) >= 1 
                max_v = 1 ;
            else 
                max_v = obj.pre_wf+obj.delta_mw ;
            end
            % UC_bond = [(bond(min_v)-u_feedback)/obj.delta,(bond(max_v)-u_feedback)/obj.delta];
            UC_bond = [bond(0.0)-u_feedback,bond(1.0)-u_feedback];
            % 
            if fmode ~= 3
                u = solve_MPC([obj.current_T-set_point;obj.current_T-set_point],fmode,UC_bond); %size(u) 2*1
                u = u(1,1);
                constant = (u-u_feedback/obj.delta)*obj.delta+u_feedback;
            else 
                A = [1 0;0 1];
                B = [1 0;0 1]; 
                Q = [1 0; 0 1]; 
                R = diag([9000,9000]);
                [k, P] =  dlqr(A, B, Q, R);
                constant = u_feedback*(k(1,1)/0.01);
            end
            % constant = u_feedback;
            % constant =  obj.next_Temp(current_density)-obj.current_T;
            fun = @(z)solve_water_flow(obj,current_density,constant,z);
            % output = fsolve(fun,0.5);
            if fun(1.0) <= 0 && fun(0.0) >= 0
                output = fsolve(fun,0.5,optimset('Display','off'));
            else
                % aduai = fun(1.0);
                % awdihawd = fun(0.0);
                output = obj.pre_wf;
                obj.fail = obj.fail+1;
            end
            obj.pre_wf = output;
        end
    end
end

