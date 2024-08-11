function [coffe,outputArg1,outputArg2,outputArg3] = propagation_pd(Acl,coffe,G,pd_1,pd_2,D)
%CONS_PROGAPD 此处显示有关此函数的摘要
%   此处显示详细说明
% samples = zeros(1000,2);
sample_1 = random(pd_1,1000,1);
sample_2 = random(pd_2,1000,1);
samples = [sample_1,sample_2];%sze(samples) = 1000*2
samples_W = (samples/coffe)*Acl'+ D*G';%size(samples_W) = 1000*2
% figure
% subplot(2,1,1)
% plot(D(:,1),D(:,2),'r+');
% subplot(2,1,2)
% plot(samples_W(:,1),samples_W(:,2),"g+")
[coffe,pd_1,pd_2] = cons_uncertainty_dis(samples_W);
outputArg1 = pd_1;
outputArg2 = pd_2;
outputArg3 = samples_W;
end

