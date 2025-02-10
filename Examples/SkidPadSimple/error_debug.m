close all; clc;

% 定义参数
r = 30; % 圆的半径
x0 = 0; % 圆心的x坐标
y0 = 0; % 圆心的y坐标
x_ = 0;
y_ = -36;
psi_ = pi / 9.0;
L = 10;
% 计算 x 坐标的增量
delta_x = L / sqrt(1 + tan(psi_)^2);
% 预瞄点的x坐标
x_preview = x_ + delta_x;
% 预瞄点的y坐标
y_preview = y_ + tan(psi_) * delta_x;

% 计算与航向角方向垂直直线的方程
psi = psi_;
if abs(abs(psi) - 0) < 1e-10 || abs(abs(psi) - pi) < 1e-10
    % 当航向角接近 0 或 π 时，垂直直线为 x = x_preview
    % 代入圆的方程 (x - x0)^2 + (y - y0)^2 = r^2 求解 y
    % 得到 (x_preview - x0)^2 + (y - y0)^2 = r^2
    % 进而 y = y0 ± sqrt(r^2 - (x_preview - x0)^2)
    discriminant = r^2 - (x_preview - x0)^2;
    if discriminant < 0
        error('直线与圆无交点');
    elseif discriminant == 0
        % 直线与圆相切，只有一个交点
        y_intersection = y0;
        intersections = [x_preview, y_intersection];
    else
        % 直线与圆相交，有两个交点
        y_intersection1 = y0 + sqrt(discriminant);
        y_intersection2 = y0 - sqrt(discriminant);
        intersections = [x_preview, y_intersection1; x_preview, y_intersection2];
    end
elseif abs(abs(psi) - pi/2) < 1e-10 || abs(abs(psi) - 3*pi/2) < 1e-10
    % 当航向角接近 π/2 或 3π/2 时，垂直直线为 y = y_preview
    % 代入圆的方程 (x - x0)^2 + (y - y0)^2 = r^2 求解 x
    % 得到 (x - x0)^2 + (y_preview - y0)^2 = r^2
    % 进而 x = x0 ± sqrt(r^2 - (y_preview - y0)^2)
    discriminant = r^2 - (y_preview - y0)^2;
    if discriminant < 0
        error('直线与圆无交点');
    elseif discriminant == 0
        % 直线与圆相切，只有一个交点
        x_intersection = x0;
        intersections = [x_intersection, y_preview];
    else
        % 直线与圆相交，有两个交点
        x_intersection1 = x0 + sqrt(discriminant);
        x_intersection2 = x0 - sqrt(discriminant);
        intersections = [x_intersection1, y_preview; x_intersection2, y_preview];
    end
else
    % 一般情况，计算垂直直线的斜率
    perpendicular_slope = -1 / tan(psi);
    % 计算过预瞄点且与航向角方向垂直的直线方程
    % 直线方程为 y - y_preview = perpendicular_slope * (x - x_preview)
    % 联立圆的方程 (x - x0)^2 + (y - y0)^2 = r^2 求解交点

    % 将直线方程变形为 y = perpendicular_slope * (x - x_preview) + y_preview
    % 代入圆的方程得 (x - x0)^2 + (perpendicular_slope * (x - x_preview) + y_preview - y0)^2 = r^2
    % 展开并整理为关于 x 的一元二次方程：aa*x^2 + bb*x + cc = 0
    aa = 1 + perpendicular_slope^2;
    bb = -2 * x0 + 2 * perpendicular_slope * (y_preview - y0 - perpendicular_slope * x_preview);
    cc = x0^2 + (y_preview - y0 - perpendicular_slope * x_preview)^2 - r^2;

    % 求解一元二次方程
    discriminant = bb^2 - 4 * aa * cc;
    if discriminant < 0
        error('直线与圆无交点');
    elseif discriminant == 0
        % 直线与圆相切，只有一个交点
        x_intersection = -bb / (2 * aa);
        y_intersection = perpendicular_slope * (x_intersection - x_preview) + y_preview;
        intersections = [x_intersection, y_intersection];
    else
        % 直线与圆相交，有两个交点
        x_intersection1 = (-bb + sqrt(discriminant)) / (2 * aa);
        y_intersection1 = perpendicular_slope * (x_intersection1 - x_preview) + y_preview;
        x_intersection2 = (-bb - sqrt(discriminant)) / (2 * aa);
        y_intersection2 = perpendicular_slope * (x_intersection2 - x_preview) + y_preview;
        intersections = [x_intersection1, y_intersection1; x_intersection2, y_intersection2];
    end
end

% 计算每个交点到预瞄点的距离
distances = sqrt((intersections(:, 1) - x_preview).^2 + (intersections(:, 2) - y_preview).^2);

% 找到距离最近的交点
[~, min_index] = min(distances);
nearest_point_x = intersections(min_index, 1);
nearest_point_y = intersections(min_index, 2);
x_A = nearest_point_x;
y_A = nearest_point_y;

% 计算点A的航向角
tangent_slope = - (x_A - x0) / (y_A - y0); % 切线斜率
heading_angle = atan(tangent_slope); % 航向角（弧度）
heading_angle_deg = rad2deg(heading_angle); % 航向角（度）

% 输出结果
fprintf('点A的坐标为 (%f, %f)\n', x_A, y_A);
fprintf('点A的航向角为 %f 度\n', heading_angle_deg);

% 绘图
% 绘制圆形路径
t = linspace(0, 2*pi, 1000);
x_circle = r * sin(t) + x0;
y_circle = - r * cos(t) + y0;
figure; hold on;
plot(x_circle, y_circle, 'b', 'DisplayName', '参考路径');

% 绘制自车初始位置
plot(x_, y_, 'ro', 'DisplayName', '自车初始位置');
quiver_len = 5;
quiver(x_, y_, cos(psi_) * quiver_len, sin(psi_) * quiver_len, 0, 'color', 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', '射线方向');

% 绘制预瞄点
plot(x_preview, y_preview, 'ko', 'DisplayName', '预瞄点');

% 绘制点A
plot(x_A, y_A, 'ko', 'DisplayName', '点A');

% 绘制过预瞄点平行于y轴的直线
line([x_preview, x_A], [y_preview, y_A], 'Color', 'r', 'LineStyle', '--');

% 绘制点A的切线方向
tangent_length = 5; % 切线长度
x_tangent = [x_A - tangent_length * cos(heading_angle), x_A + tangent_length * cos(heading_angle)];
y_tangent = [y_A - tangent_length * sin(heading_angle), y_A + tangent_length * sin(heading_angle)];
plot(x_tangent, y_tangent, 'm-', 'DisplayName', '点A的切线方向');

% 设置图形属性
axis equal;
xlabel('X');
ylabel('Y');
title('参考路径、预瞄点');
legend;
grid on;

% 预瞄点的偏差
e_d_L = -sqrt((x_preview-x_A)^2+ (y_preview-y_A)^2);
e_psi_L = psi_-heading_angle;
fprintf('预瞄点的实际偏差:\n');
[e_d_L, rad2deg(e_psi_L)]

% 质心的偏差
X = x_;
Y = y_;
PSI_ = psi_;
vx = 8.333;
vy = 0.0;
dPSI_ = 0.0;
s = x_circle;
d = y_circle;
curvature = ones(size(s)) / r;
[kr, err] = error_calc(X, Y, PSI_, vx, vy, dPSI_, s, d, t, curvature);
fprintf('质心的实际偏差:\n');
e_d = err(1);
e_psi = err(3);
[e_d, rad2deg(e_psi)]

% 用质心的偏差近似计算预瞄点的偏差
e_d_L_ = e_d + e_psi * L - 0.5 * kr * L^2;
% e_d_L_ = e_d + e_psi * L;
e_psi_L_ = e_psi - kr * L;
fprintf('用质心的实际偏差估算得到的预瞄点偏差:\n');
[e_d_L_, rad2deg(e_psi_L_)]