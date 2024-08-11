%simulation
time = 1000;
samples = time*1/delta;
temp = zeros(samples,1);
T = T_0;
for i = 1:samples
    T = delta*(-alpha*T+input(T+273,2.0)/C_t + T_a/tao_t + (C_c*T_ci/C_t)*(1-exp(-A_hxH_hx/C_c))) + T;
    temp(i,1) = T;
end
figure
plot(1:samples,temp);