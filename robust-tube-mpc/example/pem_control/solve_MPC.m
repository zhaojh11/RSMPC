function u = solve_MPC(current_x,fmode,UC_bond)
    if fmode <= 1
        mpc = evalin("base",'MPC_Tube');
        u = mpc.solve(current_x,UC_bond);
    else
        mpc = evalin("base",'MPC_SMPC');
        B = [1 0;0 1]; 
        N_horizon = 10;
        Vold = zeros(size(B,1)*(N_horizon+1)+size(B,2)*N_horizon,fmode-1);
        [u,~] = mpc.solve(current_x,Vold,fmode-1,UC_bond);
    end
    % B = [1 0;0 1]; 
    % N_horizon = 10;
    % Vold = zeros(size(B,1)*(N_horizon+1)+size(B,2)*N_horizon,fmode);
    % [u,~] = mpc.solve(current_x,Vold,fmode,UC_bond);
end