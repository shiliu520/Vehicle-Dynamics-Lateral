% 定义系统参数
% \ddot e + \frac{2\dot e}{T} + \frac{2e}{T^2}=0 % T = 2
% \ddot e + 2 * wn * zeta \dot e + wn^2 e = 0
% \ddot e + \dot e + 1 / 2 e = 0
wn = sqrt(2) / 2; % 无阻尼自然频率
zeta = sqrt(2) / 2; % 阻尼比
k = 1;
wn = k;
zeta = 1;

% 定义初始条件
y0 = 1; % y(0)
dy0 = 0; % dy/dt(0)

% 定义时间范围
t = 0:0.01:10;

% 定义系统状态空间方程
A = [0 1; -wn^2 -2*zeta*wn];
B = [0; 0];
C = [1 0];
D = 0;
sys = ss(A, B, C, D);

% 进行零输入响应仿真
y = initial(sys, [y0; dy0], t);

% 绘制响应曲线
plot(t, y);
xlabel('时间 (s)');
ylabel('输出');
title('输入为零且有初值的标准二阶系统的响应');
grid on;