clear;
%固定种子
rng(0);

%电解槽的温度控制
tic;

%设置控制时间，温度限制，实验次数
time = 2500;
set_point = 80;
naf = zeros(time,1);
awac = zeros(time,1);
realization = 50;

%温度控制
tube_matrix = zeros(time,1);
smpc_matrix = zeros(time,1);
lqr_matrix = zeros(time,1);
pid_matrix = zeros(time,1);
v_smpc = 0;
v_lqr = 0;
v_tube = 0;
v_pid = 0; 
c_smpc = 0;
c_lqr = 0;
c_tube = 0;
c_pid = 0;
for j = 3:3
    %构建tube-based MPC控制器以及所提出的方法的控制器
    if j == 1
        addpath('../../../origin_MPC/robust-tube-mpc/src/')
        addpath('../../../origin_MPC/robust-tube-mpc/src/utils/')
        rmpath('../../src/')
        rmpath('../../src/utils/')
        % path;
        MPC_Tube = construct_Tube();
    else
        clear MPC_Tube;
        addpath('../')
        addpath('../../src/')
        addpath('../../src/utils/')
        rmpath('../../../origin_MPC/robust-tube-mpc/src/')
        rmpath('../../../origin_MPC/robust-tube-mpc/src/utils/')
        % path;
        MPC_SMPC = construct_SMPC();
    end
    for rel = 1:realization
        % disp(rel);
        % disp(c_smpc);
        pidc = pidcontroller(80);
        temp = Theramalmodel(30);
        for i = 1:time
        %2.0是指提供稳定的电流密度
        %m_w是求解出的水流速率
            m_w = temp.solve_wf(2.0,j,set_point); 
            % disp(m_w);
            % m_w = pidc.pidcontrol(temp.current_T);
            ne_T = temp.next_Temp(2.0,m_w);
            naf(i,1) = ne_T;
            awac(i,1) = m_w;
        end
        % disp(temp.fail);
        if j == 2
            v_smpc = v_smpc+ sum(naf>set_point);
            c_smpc = c_smpc+ sum(naf>78.5);
        elseif j == 3
            v_lqr = v_lqr+ sum(naf>set_point);
            c_lqr = c_lqr+ sum(naf>78.5);
        elseif j == 4
            v_pid = v_pid+ sum(naf>set_point);
            c_pid = c_pid+ sum(naf>78.5);
        else
            v_tube = v_tube+ sum(naf>set_point);
        end

%可以将控制结果保存下来，方便画图
        if j ==1
            tube_matrix = [tube_matrix,naf];
        elseif j == 2
            smpc_matrix = [smpc_matrix,naf];
        elseif j == 4
            pid_matrix = [pid_matrix,naf];
        else
            lqr_matrix = [lqr_matrix,naf];
        end
    end
end

%这里应该计算的是算法收敛时候的约束违反率，对于SMPC以及LQR的收敛情况，就简单地判断为到达78.5摄氏度时。
disp(v_smpc/c_smpc);
disp(v_lqr/c_lqr);
disp(v_tube/realization);
disp(v_pid/c_pid)

toc
disp(['运行时间: ',num2str(toc)]);