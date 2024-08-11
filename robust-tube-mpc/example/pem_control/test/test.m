R = 8.314;
alpha_a = 0.5;
alpha_c = 0.5;
z = 2;
F = 96485.33289;
% i_c = 0;
i_o_c = 1E-3;
% i_a = 0;
i_o_a = 3E-5;
R_ele = 3.5E-5;
t_mem = 1.27E-2;
lamda = 12.7;

Area = 170;
NC = 100;
V_th = 1.48;
syms i;
% C_p = 42045;
for j = 1:5
    if j ==1
        temp = 30+273;
    elseif j ==2
        temp = 40+273;
    elseif j ==3
        temp = 50+273;
    elseif j ==4
        temp = 60+273;
    elseif j ==5
        temp = 70+273;
    end
    
V_rev = 1.5184 - 1.5421E-3*temp+9.523E-5*temp*log(temp)+9.84E-8*temp^2;
R_mem = t_mem/((0.005139*lamda - 0.00326)*exp(1268*(1/303-1/temp)));
v_cell(i) = (V_rev + (R*temp/(alpha_a*z*F))*log(i/i_o_a) + (R*temp/(alpha_c*z*F))*log(i/i_o_c) + (R_ele+R_mem)*i);
fplot(v_cell,[0.01,2.5],LineWidth=2);
hold on;
end