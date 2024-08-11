function output = solve_wf(T)
    u = solve_MPC(T); %size(u) 2*1
    u = u(1,1);
    
    function y = solve_water_flow(constant,T,m_w)
        C_pc = 4180;
        C_c = m_w*C_pc; 
        T_ci = 20;
        C_t = 42045;
        A_hxH_hx = 380;
        y = -(C_c/C_t)*(1-exp(-A_hxH_hx/C_c))*T + (C_c*T_ci/C_t)*(1-exp(-A_hxH_hx/C_c))+constant;
    end
    fun = @(z)solve_water_flow(constant,T,z);
    output = fsolve(fun,0.5);
end