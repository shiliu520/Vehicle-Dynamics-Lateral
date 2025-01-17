% Linear tire model
% close all;
Kf = 3.0054e+04;
Kr = 2.8930e+04;
u = 8.333;
K = System.mT / System.lT^2 * (System.a / (-Kr) - System.b / (-Kf));
omega_r = u / System.lT / (1 + K * u^2) * System.deltaf(end);
beta_s1 = (System.a * (-Kf) * System.deltaf(end) - omega_r / u * (System.a^2 * (-Kf) + System.b^2 * (-Kr))) / (System.a * (-Kf) - System.b * (-Kr));
beta_s2 = (System.mT * u * omega_r -Kf * System.deltaf(end) - omega_r / u * (System.a * (-Kf) - System.b * (-Kr))) / (-Kf - Kr);
beta_s3 = System.deltaf(end) / (1 + K * u ^2) * (System.b / System.lT + System.mT * System.a * u^2 / (-Kr) / System.lT^2 );
fprintf('steady beta linear mode [rad]: %f\n', beta_s3);
fprintf('steady yaw rate linear mode [rad/s]: %f\n', omega_r);

%% state space simulation
Caf = Kf;
Car= Kr;
Cf = Caf;
Cr = Car;
m = System.mT;
a = System.a;
b = System.b;
Iz = System.IT;
L = a + b;

% 输入
t=TSPAN;
U=System.deltaf;

% 计算状态空间矩阵
A1 = [- (Cf + Cr)/(m*u), - (a*Cf - b*Cr)/(m*u^2) - 1;
    - (a*Cf - b*Cr)/Iz, - (a^2*Cf + b^2*Cr)/(Iz*u)];
B1 = [Cf/(m*u); a*Cf/Iz];
C1 = eye(2); % state is [Side slip angle; Yaw rate]
D1 = zeros(2,1);
sys1 = ss(A1,B1,C1,D1);
[y1,t1,x1]=lsim(sys1,U,t);
dx1 = A1 * x1' + B1 * U;
a1 = u * (dx1(1,:) + y1(:, 2)');

A2=[-(Caf+Car)/(m*u), (b*Car-a*Caf)/(m*u)-u;
    (b*Car-a*Caf)/(Iz*u), -(a^2*Caf+b^2*Car)/(Iz*u)];
B2=[Caf/m; a*Caf/Iz];
C_lat = [1 0]; D_lat = 0; % state is [Lateral speed, Yaw rate]
C_yaw = [0 1]; D_yaw = 0;
C_acc=A2(1,:) + u*[0,1];
D_acc = B2(1); % Lateral acceleration
C = [C_lat; C_yaw; C_acc];
D = [D_lat; D_yaw; D_acc];

sys2 = ss(A2,B2,C,D);
[y2,t2,x2]=lsim(sys2,U,t);

%% input: steer output: wr
m_ = m * u * Iz;
h_ = m * (a^2 * Cf + b^2 * Cr) + Iz * (Cf + Cr);
c_ = m * u * (a * (-Cf) - b * (-Cr)) + L^2 * Cf * Cr / u;
b1 = m * u * a * Cf;
b0 = L * Cf * Cr;
num = [b1, b0];
den = [m_, h_, c_];
tf1 = tf(num, den);
[omega,t3]=lsim(tf1,U,t);
num1 = [Iz, (a^2 * Cf + b^2 * Cr) / u];
den1 = [a*(-Cf) - b * (-Cr)];
tf2 = tf(num1, den1);
num3 = [0, a * (-Cf)];
den3 = [0, a*(-Cf) - b * (-Cr)];
tf3 = tf(num3, den3);
tf_omega_beta = tf2 * tf1 + tf3;
[beta1,t4]=lsim(tf_omega_beta,U,t);
num_omega_beta = conv(num1, num) + conv(num3(2), den);
den_omega_beta = conv(den1, den);
tf_omega_beta1 = tf(num_omega_beta, den_omega_beta);
[beta2,t5]=lsim(tf_omega_beta1,U,t);

diff_link = tf([1, 0], [0, 1]);
tf_delta_a = u * (tf_omega_beta * diff_link + tf1);
[ay,t6]=lsim(tf_delta_a,U,t);

%% results plot
stack_info = dbstack;
if length(stack_info) > 1
    plot_res = 0;
else
    plot_res = 1;
end
if plot_res == 1
    close all;
    figure(11)
    subplot(221);
    plot(t,y2(:,1),'r'); grid
    xlabel('time (sec)')
    ylabel('Lateral speed (m/sec)')
    subplot(222)
    plot(t,y2(:,2),'r'); grid
    xlabel('time (sec)')
    ylabel('Yaw rate (rad/sec)')
    subplot(223)
    plot(t,y2(:,3),'r'); grid
    xlabel('time (sec)')
    ylabel('Lat. Accel.(m/sec^2)')
    subplot(224)
    plot(t,U*180/pi,'r'); grid
    xlabel('time (sec)')
    ylabel('Steering (deg)');
    
    figure(22)
    subplot(221);
    plot(t,y1(:,1),'r'); grid
    xlabel('time (sec)')
    ylabel('side slip angle (rad/s)')
    subplot(222)
    plot(t,y1(:,2),'r'); grid
    xlabel('time (sec)')
    ylabel('Yaw rate (rad/sec)')
    subplot(223)
    plot(t,a1','r'); grid
    xlabel('time (sec)')
    ylabel('Lat. Accel.(m/sec^2)')
    subplot(224)
    plot(t,U*180/pi,'r'); grid
    xlabel('time (sec)')
    ylabel('Steering (deg)');
    
    
    figure('Position', [100, 100, 1500, 400]);
    subplot(132)
    plot(t,omega,'r'); grid
    xlabel('time (sec)')
    ylabel('Yaw rate \omega (rad/sec)');
    subplot(131)
    hold on;
    plot(t,beta1,'r-o'); plot(t,beta2,'b-+');
    grid
    xlabel('time (sec)')
    ylabel('Side slip angle \beta (rad/sec)');
    subplot(133);
    plot(t,ay,'r'); grid
    xlabel('time (sec)')
    ylabel('Lat. Accel.(m/sec^2)')
end