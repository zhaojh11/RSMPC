function [X_poly] = cons_uncertainty_poly(W,epsilon,xmax,xmin)
%CONS_UNCERTAINTY_POLY 此处显示有关此函数的摘要
%   此处显示详细说明
x_vertex = [xmax(1,1),xmax(2,1);xmax(1,1),xmin(2,1);xmin(1,1),xmin(2,1);xmin(1,1),xmax(2,1)];
X_poly = Polyhedron(x_vertex);
% X_poly.plot();
[bandwidth,density,X,Y]=kde2d(W);
sum_1 = sum(density,1)/sum(density(:));
sum_2 = sum(density,2)/sum(density(:));
num_1 = 0;num_2=0;num_3=0;num_4=0;
epsilon = 0.4;
for idx_1  = 1:size(density,1)
    if num_1 >= epsilon
        break;
    end
    num_1 = num_1+sum_1(1,idx_1);
end
for idx_2  = 1:size(density,1)
    if num_2 >= epsilon
        break;
    end
    num_2 = num_2+sum_2(idx_2,1);
end
for idx_3  = 1:size(density,1)
    if num_3 >= 1-epsilon
        break;
    end
    num_3 = num_3+sum_1(1,idx_3);
end
for idx_4  = 1:size(density,1)
    if num_4 >= 1-epsilon
        break;
    end
    num_4 = num_4+sum_2(idx_4,1);
end
x_1 = X(1,idx_1);x_2 = X(1,idx_3);y_1 = Y(idx_2,1);y_2 = Y(idx_4,1);
% x_max = pd_1.icdf(1-epsilon);
% x_min = pd_1.icdf(epsilon);
% y_max = pd_2.icdf(1-epsilon);
% y_min = pd_2.icdf(epsilon);
% D_polyvex = [x_max,y_max;x_max,y_min;x_min,y_min;x_min,y_max];
% D_poly = Polyhedron(D_polyvex);
% W_ploy = D_poly*inv(coffe);
W_vex =[x_2,y_2;x_2,y_1;x_1,y_1;x_1,y_2];
W_ploy = Polyhedron(W_vex);
% figure
% W_ploy.plot();
X_poly = X_poly-W_ploy;
% outputArg1 = inputArg1;
% outputArg2 = inputArg2;
end

