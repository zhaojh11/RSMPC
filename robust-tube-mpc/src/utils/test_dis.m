clear;
mu = [0 0];
SIGMA = [0.5 -0.3; -0.3 0.5];
SIGMA = [0.08^2,-0.005;-0.005,0.08^2];
r = mvnrnd(mu,SIGMA,10000);
% plot(r(:,1),r(:,2),'r+');

[coffe,pd_1,pd_2] = cons_uncertainty_dis(r);
% 
% [coffe,sco] = pca(r);
% W_R = 1/999*(r'*r);
% co = inv(coffe);
% score = coffe\W_R/coffe';
% sco_ = (r-mean(r))*coffe;
% % [band,density,X,Y] = kde2d(sco_);
% % contour3(X,Y,density,50), hold on
% % plot(sco_(:,1),sco_(:,2),'r.','MarkerSize',5)
% pd_1 = fitdist(sco_(:,1),"Kernel");
% % pd_2 = fitdist(sco_(:,2),"Kernel");
% % covpdf = conv(pd_1,pd_2);
% x_min = [-10;-10];
% x_max = [0.15;0.15];
x_vertex = [0.15,0.15;0.15,-10;-10,-10;-10,0.15];
X_poly = Polyhedron(x_vertex);
% X_poly.plot();
x_max = pd_1.icdf(0.95);
x_min = pd_1.icdf(0.05);
y_max = pd_2.icdf(0.95);
y_min = pd_2.icdf(0.05);
D_ployvex = [x_max,y_max;x_max,y_min;x_min,y_min;x_min,y_max];
D_ploy = Polyhedron(D_ployvex);
W_ploy = D_ploy*inv(coffe);
W_ploy1 = inv(coffe)*D_ploy;
figure
subplot(3,1,1)
afX = X_poly-W_ploy;
afX.plot();
subplot(3,1,2)
plot(r(:,1),r(:,2),'r+');
subplot(3,1,3)
W_ploy.plot();
% xsad = random(pd_1,1000,1);
% pd_2 = fitdist(xsad,'Kernel');
% figure
% subplot(2,1,1)
% x = -5:0.02:5;
% y = pdf(pd_1,x);
% plot(x,y,'LineWidth',2)
% title('Miles per Gallon')
% xlabel('MPG')
% subplot(2,1,2)
% % histfit(xsad,100)
% x = -5:0.02:5;
% y = pdf(pd_2,x);
% plot(x,y,'LineWidth',2)
% title('Miles per Gallon')
% xlabel('ax')
% 
% % % co*coffe
