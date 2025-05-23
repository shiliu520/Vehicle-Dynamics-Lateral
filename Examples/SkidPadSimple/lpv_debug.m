% 假设我们有一个简单的 LPV 系统的数据
% y(k) = a(k)*y(k-1) + b(k)*u(k) + c
% 其中 c 是可能未变化的参数

% 生成模拟数据
N = 1000; % 数据点数
u = randn(N,1); % 输入信号
y = zeros(N,1); % 输出信号
a = 0.8*randn(N,1); % 变化的参数 a
b = 0.5*randn(N,1); % 变化的参数 b
c = 2; % 假设未变化的参数

y(1) = c + a(1)*0 + b(1)*u(1); % 初始条件
for k = 2:N
    y(k) = a(k)*y(k-1) + b(k)*u(k) + c;
end

% 使用系统辨识工具箱
data = iddata(y,u);

% 尝试使用 ARX 模型进行初步辨识
model_arx = arx(data,[1 1 0]); % [na nb] 这里假设 na = 1, nb = 1

% 查看辨识结果
disp(model_arx);

% 进一步分析结果，判断哪些参数相对稳定
% 可以通过多次改变数据生成条件，重新运行上述代码
% 观察不同次辨识结果中参数的变化情况