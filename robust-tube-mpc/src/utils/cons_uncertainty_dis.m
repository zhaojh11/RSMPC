%不确定性数据处理函数
function [coffe,pd_1,pd_2] = cons_uncertainty_dis(D)
    % 不确定性数据集D
    % 主成分系数矩阵coffe
    % 主成分估计分布pd_1,pd_2....
% clear;
%     mu = [0 0];
%     SIGMA = [0.5 -0.3; -0.3 0.5];
%     r = mvnrnd(mu,SIGMA,1000);
%     plot(r(:,1),r(:,2),'r+');
    
    [coffe,sco] = pca(D);
%     W_R = 1/999*(r'*r);
%     co = inv(coffe);
%     score = coffe\W_R/coffe';
%     sco_ = (D-mean(D))*coffe;
    pd_1 = fitdist(sco(:,1),"Kernel");
    pd_2 = fitdist(sco(:,2),"Kernel");
    % covpdf = conv(pd_1,pd_2);
    % cdf = pd_1.icdf(true);
%     xsad = random(pd_1,1000,1);
%     pd_2 = fitdist(xsad,'Kernel');
%     figure
%     subplot(2,1,1)
%     x = -5:0.02:5;
%     y = pdf(pd_1,x);
%     plot(x,y,'LineWidth',2)
%     title('Miles per Gallon')
%     xlabel('MPG')
%     subplot(2,1,2)
%     % histfit(xsad,100)
%     x = -5:0.02:5;
%     y = pdf(pd_2,x);
%     plot(x,y,'LineWidth',2)
%     title('Miles per Gallon')
%     xlabel('ax')
    
    % % co*coffe
