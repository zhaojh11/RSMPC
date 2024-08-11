clear;
fig = openfig('C:\Users\12911\Desktop\tubepem.fig');
fig2 = openfig('C:\Users\12911\Desktop\untitled.fig');
fig3 = openfig('C:\Users\12911\Desktop\feedback.fig');
sample = 1;
% 找到图形对象，根据需求自动调整参数
lines = findobj(fig, 'type', 'line');


% 获取数据
xdata = get(lines, 'XData');
ydata = get(lines, 'YData');

lines1 = findobj(fig2, 'type', 'line');

% 获取数据
xdata1 = get(lines1, 'XData');
ydata1 = get(lines1, 'YData');
lines2 = findobj(fig3, 'type', 'line');

% 获取数据
xdata2 = get(lines2, 'XData');
ydata2 = get(lines2, 'YData');
num = 0;
for i = 1:sample
    data = ydata(i,1);
    darta_ = data{1,1};
    num = num + sum(darta_);
    % a = sum(ydata(i,1));
end
num = num/sample;
num1 = 0;
for i = 1:sample
    data = ydata1(i,1);
    darta_ = data{1,1};
    num1 = num1 + sum(darta_);
    % a = sum(ydata(i,1));
end
num1 = num1/sample;
num2 = 0;
for i = 1:sample
    data = ydata2(i,1);
    darta_ = data{1,1};
    num2 = num2 + sum(darta_);
    % a = sum(ydata(i,1));
end
num2 = num2/sample;
