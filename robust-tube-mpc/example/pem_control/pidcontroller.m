classdef pidcontroller < handle
    properties
        kp = -100;
        ki = 1e-4;
        kd = 0;
        ts = 0.001;   %时间参数
        setpoint;      %期望值
        u_min = 0;
        u_max = 1;
        e;   %误差信号
        % u(k)=kp*e(k)+ki*Ee+kd*(e(k)-e_1); %系统PID控制器输出序列
        Ee;    %误差的累加和
        u_1;    	%前一个的控制器输出值
        y_1;    	%前一个的系统响应输出值
        e_1;		%前一个误差信号的值
    end
    methods
        function obj = pidcontroller(setpoint)
            obj.setpoint = setpoint;
            obj.e = 0;
            obj.Ee = 0;    %误差的累加和
            obj.u_1=0;    	%前一个的控制器输出值
            obj.y_1=0;    	%前一个的系统响应输出值
            obj.e_1=0;		%前一个误差信号的值
        end
        function [u,obj] = pidcontrol(obj,input)
            obj.e = obj.setpoint-input;
            u = obj.kp*obj.e+obj.ki*obj.Ee+obj.kd*(obj.e-obj.e_1);
            obj.Ee = obj.Ee+obj.e;
            obj.e_1 = obj.e;
            if u < obj.u_min
                u = obj.u_min;
            end
            if u > obj.u_max
                u = obj.u_max;
            end
        end
    end
end

