# PEM电解槽建模与控制
## PEM电解槽的建模
Theramalmodel.m是电解槽的热力学建模，并且该类中包含了求解控制水流流速的函数。
## PEM电解槽的温度控制
控制的代码流程是：直接运行the_whole_control.m脚本文件即可。
该脚本的运行逻辑是先构建用于控制电解槽的控制器，再进行电解槽的温度建模，在调用温度建模中的求解控制水流函数solve_water，最后在该水流下给出下一时刻的电解槽温度。

其中construct_SMPC以及construct_Tube是在之前完成的算法中稍作修改后，用来构建应用于PEM电解槽温度控制的控制器。
construct_SMPC需要的类是‘../../src/’中的TubeModelPredictiveControl，而construct_Tube需要的类是'../../../origin_MPC/robust-tube-mpc/src/'中的TubeModelPredictiveControl(运行时可能会有搜索路径的问题)

而LQR一轮实验是3.5秒左右，Tube MPC以及Proposed MPC每一轮实验9秒左右，所以运行100轮实验需要的时间较长。
画效果图：运行完the_whole_control.m后保存工作区里面的smpc_matrix,lqr_matrix以及tube_matrix，再运行make_fig.m脚本画效果图。
