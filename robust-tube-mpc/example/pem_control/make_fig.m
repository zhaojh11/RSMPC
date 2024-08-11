realization = 50;
time =2500;
f1 =  figure(1);
for j = 1:4
    if j == 1
        matrix = lqr_matrix;
        color = [0 1 0];
    elseif j == 2
        matrix = tube_matrix;
        color = [0 0 1];
    elseif j == 4
        matrix = pid_matrix;
        color = [0 0 1];
    else
        matrix = smpc_matrix;
        color = [1 0 0];
    end
    for i = 2:realization+1
        if j == 2
            if sum(matrix(:,i)>75) <= 1000
                h1 = plot(1:time,matrix(:,i),'r:');
                h1.LineWidth = 2;
                hold on;
            end
        elseif j ==3
            h2 = plot(1:time,matrix(:,i),'b--');
            h2.LineWidth = 2;
            hold on;
        elseif j ==4
            h4 = plot(1:time,matrix(:,i),'g-');
            h4.LineWidth = 2;
            hold on;
        else 
            h3 = plot(1:time,matrix(:,i),'k-.');
            h3.LineWidth = 2;
            hold on;
        end
    end
end
line([0,2500],[80,80],'linestyle','-','color','k');
hold on;
% box on;
% h1.LineWidth = 2;
% h2.LineWidth = 2;
% h3.LineWidth = 2;
% h4.LineWidth = 2;
legend([h1,h3,h2,h4],'The tube-based MPC','LQR-based approach','The proposed approach','PID control','Location','southeast','FontSize', 24,'LineWidth',2);
% % I=imread("C:\Users\12911\Desktop\专利图片.png");
% % 图片维度
% % [m,n]=size(I);
% % 原图
% % figure(1);
% % imshow(I);
% % figure(2);
% % I1=rgb2gray(I);
% % 将RGB图像转换为灰度图
% % imshow(I1);
% % %黑白图
% % figure(3);
% % %imshow(I,[low high]) 显示灰度图像 I，以二元素向量 [low high] 形式指定显示范围
% % imshow(I1,[100 200]);
% % %保存灰度图
% % imwrite(I1,'灰度微笑.png')
