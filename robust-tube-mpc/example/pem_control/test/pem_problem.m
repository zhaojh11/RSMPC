clear;
% pem热模型 cite:Modeling for Optimal Operation of PEM Fuel Cells and Electrolyzers
%cell voltage
function x = input(next_T,i)
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
    % C_p = 42045;

    V_rev = 1.5184 - 1.5421E-3*next_T+9.523E-5*next_T*log(next_T)+9.84E-8*next_T^2;
    R_mem = t_mem/((0.005139*lamda - 0.00326)*exp(1268*(1/303-1/next_T)));
    v_cell = (V_rev + (R*next_T/(alpha_a*z*F))*log(i/i_o_a) + (R*next_T/(alpha_c*z*F))*log(i/i_o_c) + (R_ele+R_mem)*i-V_th);
    x =v_cell*i*Area*NC;
end


fun = @(x)input(50+273,x);
fplot(fun,[0.1,2]);
% input(50+273,2.0)/42045
% % parameters
% C_t = 42045;
% C_pc = 4180;
% A_hxH_hx = 380;
% R_t = 0.082;
% T_0 = 25;
% T_a = 25;
% T_ci = 20;
% N = 100;
% I = 170;
% U_th = 1.48;
% U = 2.0;
% delta = 1.0;
% 
% tao_t = R_t*C_t;
% m_c = 0.25; 
% C_c = m_c*C_pc; 
% 
% % model
% function x = next_temp(current_T,current_density)
%     C_t = 42045;
%     C_pc = 4180;
%     A_hxH_hx = 380;
%     R_t = 0.082;
%     % T_0 = 25;
%     T_a = 25;
%     T_ci = 20;
%     % N = 100;
%     % I = 170;
%     % U_th = 1.48;
%     % U = 2.0;
%     delta = 1.0;
% 
%     tao_t = R_t*C_t;
%     m_c = 0.25; 
%     C_c = m_c*C_pc; 
%     alpha = 1/tao_t + (C_c/C_t)*(1-exp(-A_hxH_hx/C_c));
%     belta = input(current_T+273,current_density)/C_t + T_a/tao_t + (C_c*T_ci/C_t)*(1-exp(-A_hxH_hx/C_c));
%     x = (-alpha*current_T+belta)*delta + current_T; 
% end
% 
% % class
% classdef Theramal_model < handel
