rng(0)
tic;
% temp = Theramalmodel(30);
% m_w = temp.solve_wf(2.0);
time = 2500;
set_point = 80;
naf = zeros(time,1);
awac = zeros(time,1);
realization = 50;
tube_matrix = zeros(time,1);
smpc_matrix = zeros(time,1);
lqr_matrix = zeros(time,1);
s_smpc = 0;
s_lqr = 0;
s_tube = 0;
for rel = 1:realization
    % disp(rel)
    for j = 1:3
        temp = Theramalmodel(30);
        if j == 1
            color = [1 0 0];
        elseif j == 2
            color = [0 1 0];
        else
            color = [0 0 1];
        end
        for i = 1:time
            % if i == 378
            %        disp("break");
            % end
            m_w = temp.solve_wf(2.0,j,set_point);  
            ne_T = temp.next_Temp(2.0,m_w);
            naf(i,1) = ne_T;
            awac(i,1) = m_w;
        end
        % disp(temp.fail);
        if j == 2
            s_smpc = s_smpc+ sum(naf>set_point);
        elseif j == 3
            s_lqr = s_lqr+ sum(naf>set_point);
        else
            s_tube = s_tube+ sum(naf>set_point);
        end

        % f1 =  figure(1);
        if j ==1
            tube_matrix = [tube_matrix,naf];
        elseif j ==2
            smpc_matrix = [smpc_matrix,naf];
        else
            lqr_matrix = [lqr_matrix,naf];
        end
        % if j == 1
        %     % if sum(naf>75) <= 500
        %     %     % set(f1,'Visible','off');
        %     %     subplot(3,2,1);
        %     %     line([0,time],[set_point,set_point],'linestyle','--');
        %     %     h1 = plot(1:time,naf(:,1),Color=color);
        %     %     hold on;
        %     %     subplot(3,2,2);
        %     %     plot(1:time,awac(:,1));
        %     %     hold on;
        %     %     legend(h1,'The tube-based approach','Location','SouthEast','FontSize', 14)
        %     % end
        % elseif j == 2
        % subplot(3,2,3)
        % % plot(1:time,awac(:,1));
        % line([0,time],[set_point,set_point],'linestyle','--');
        % h2 = plot(1:time,naf(:,1),Color=color);
        % hold on;
        % subplot(3,2,4)
        % plot(1:time,awac(:,1));
        % hold on;
        % legend(h2,'The proposed approach','Location','SouthEast','FontSize', 14)
        % else
        % subplot(3,2,5)
        % % plot(1:time,awac(:,1));
        % line([0,time],[set_point,set_point],'linestyle','--');
        % h3 = plot(1:time,naf(:,1),Color=color);
        % hold on;
        % subplot(3,2,6)
        % plot(1:time,awac(:,1));
        % hold on; 
        % legend(h3,'simple feedback control','Location','SouthEast','FontSize', 14)
        % end
    end
end
disp(s_smpc/realization);
disp(s_lqr/realization);
disp(s_tube/realization);
% figure(1);
% subplot(2,1,1)
% plot(1:time,naf(:,1));
% subplot(2,1,2)
% plot(1:time,awac(:,1));

toc
disp(['运行时间: ',num2str(toc)]);