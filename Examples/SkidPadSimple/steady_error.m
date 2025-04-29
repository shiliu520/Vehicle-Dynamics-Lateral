clc
syms a b cf cr m vx Iz k1 k2 k3 k4 kappa delta Caf Car L

A=[0,1,0,0;
    0,(cf+cr)/(m*vx),-(cf+cr)/m,(a*cf-b*cr)/(m*vx);
    0,0,0,1;
    0,(a*cf-b*cr)/(Iz*vx),-(a*cf-b*cr)/Iz,(a*a*cf+b*b*cr)/(Iz*vx)];
B1=[0;
    -cf/m;
    0;
    -a*cf/Iz];
B2=[0;
    (a*cf-b*cr)/m/vx-vx;
    0;
    (a^2*cf+b^2*cr)/Iz/vx];
K = [k1, k2, k3, k4];
x_ss = -inv(A - B1 * K) * (B1 * delta + B2 * vx * kappa);
x_ss = simplify(x_ss);
x_ss = collect(x_ss, delta);
x_ss;

% Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking
% p43 tire stiffness is positive, and only for one tire
kv = b * m / 1 / (-Caf) / (a + b) - a * m / 1 / (-Car) / (a + b);
ay = vx^2 * kappa;
delta_ff_paper = kappa * (a + b) + kv * ay - k3 * kappa * (b - a / 1 / (-Car) * m * vx^2 / (a + b));
% https://github.com/shiliu520/automated-driving-control/tree/main/notebook
% chapter 6 p4 tire stiffness is negative, and for two tire
delta_ff_wang = kappa * (a + b - b * k3 - m * vx^2 / (a + b) * (b / Caf + a / Car * k3 - a / Car) );

ed_ss = x_ss(1);
solutions = solve(ed_ss == 0, delta);
fprintf('output zeros shows delta_ff is correct.\n');
simplify(delta_ff_paper - forword_angle)
solutions = subs(solutions, cf, Caf);
solutions = subs(solutions, cr, Car);
fprintf('output zeros shows delta_ff can make steady ed 0.\n');
simplify(solutions - forword_angle)

%% 计算误差模型的稳态误差
% delta 前面的系数，计算出来是1/k1
simplify(((b*cr - a*cf + a*cf*k3)/(cr*k1*(a + b)) + (a*(cf + cr - cf*k3))/(cr*k1*(a + b))));
% 对剩余的部分，优先提出kappa
collect(((b*cr - a*cf + a*cf*k3)*(kappa*m*vx^2 - a*cf*kappa + b*cr*kappa))/(cf*cr*k1*(a + b)) - ((cf*kappa*a^2 + cr*kappa*b^2)*(cf + cr - cf*k3))/(cf*cr*k1*(a + b)), kappa);
% 继续提出k1
collect((((b*cr - a*cf + a*cf*k3)*(m*vx^2 - a*cf + b*cr))/(cf*cr*k1*(a + b)) - ((cf*a^2 + cr*b^2)*(cf + cr - cf*k3))/(cf*cr*k1*(a + b))), 1/k1);
% vx = 0
simplify((((b*cr - a*cf + a*cf*k3)*(m*0^2 - a*cf + b*cr))/(cf*cr*(a + b)) - ((cf*a^2 + cr*b^2)*(cf + cr - cf*k3))/(cf*cr*(a + b)))/k1);

A11 = [-(Caf+Car)/(m*vx), (b*Car-a*Caf)/(m*vx)-vx;
    (b*Car-a*Caf)/(Iz*vx), -(a^2*Caf+b^2*Car)/(Iz*vx)];
A12 = zeros(2, 2);
A21 = [-1, -L;
       0, -1];
A22 = [0, vx;
       0, 0];
B11 = [Caf/m; a*Caf/Iz];
B21 = zeros(2, 1);
A = [A11, A12;
    A21, A22];
B1 = [B11;
    B21];
B2 = [zeros(3, 1);
    vx];

%% 计算预瞄动力学模型的稳态误差
% https://tas60zbogq.feishu.cn/docx/UsFgdVjGVojjsgxs0IFcaTvnntb
% add forward preview
x_ss = -inv(A - B1 * K) * (B1 * delta + B2 * kappa);
x_ss = simplify(x_ss);
ed_ss = x_ss(3);
solutions = solve(ed_ss == 0, delta);
delta_ff = kappa * (a + b) + kappa * k4 * (b + L) + kappa * vx * (k2 + b * k1) + kappa * m * vx^2 / (Caf * Car * (a + b)) * (-a *Caf * (1 + vx * k1 + k4) + b * Car) ;

simplify(solutions - delta_ff)