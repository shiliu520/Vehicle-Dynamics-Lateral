clc;

path_selector = 2;

if path_selector == 1
    % 定义参数
    lane_width = 3.75; % 高速公路标准车道宽度（米）
    total_distance = 250; % 总行驶距离（米），加上前面 50m 直线距离
    pre_straight_distance = 50; % 前面直线行驶的距离（米）
    change_distance = 50; % 每次换道的距离（米）
    keep_distance = 50; % 在目标车道保持的距离（米）
    
    % 纵向距离数组
    s = 0:0.1:total_distance;
    
    % 初始化横向位移数组
    d = zeros(size(s));
    
    % 生成五次多项式系数（向左换道）
    % 初始条件：d(0) = 0, d'(0) = 0, d''(0) = 0
    % 最终条件：d(change_distance) = lane_width, d'(change_distance) = 0, d''(change_distance) = 0
    A = [0, 0, 0, 0, 0, 1;
        change_distance^5, change_distance^4, change_distance^3, change_distance^2, change_distance, 1;
        5*change_distance^4, 4*change_distance^3, 3*change_distance^2, 2*change_distance, 1, 0;
        20*change_distance^3, 12*change_distance^2, 6*change_distance, 2, 0, 0;
        0, 0, 0, 0, 1, 0;
        0, 0, 0, 2, 0, 0];
    b = [0; lane_width; 0; 0; 0; 0];
    coeff_left = A \ b;
    
    % 生成五次多项式系数（向右换道）
    % 初始条件：d(0) = lane_width, d'(0) = 0, d''(0) = 0
    % 最终条件：d(change_distance) = 0, d'(change_distance) = 0, d''(change_distance) = 0
    b = [lane_width; 0; 0; 0; 0; 0];
    coeff_right = A \ b;
    
    % 生成全局路径
    for i = 1:length(s)
        if s(i) <= pre_straight_distance
            % 前面直线行驶阶段
            d(i) = 0;
        elseif s(i) <= pre_straight_distance + change_distance
            % 向左换道阶段
            d(i) = polyval(coeff_left, s(i) - pre_straight_distance);
        elseif s(i) <= pre_straight_distance + change_distance + keep_distance
            % 在目标车道保持阶段
            d(i) = lane_width;
        elseif s(i) <= pre_straight_distance + change_distance + keep_distance + change_distance
            % 向右换道阶段
            d(i) = polyval(coeff_right, s(i) - (pre_straight_distance + change_distance + keep_distance));
        else
            % 回到原车道后保持
            d(i) = 0;
        end
    end
    
    % 计算航向角和曲率
    heading_angle = zeros(size(s));
    curvature = zeros(size(s));
    
    for i = 1:length(s)
        if s(i) <= pre_straight_distance
            % 前面直线行驶阶段
            heading_angle(i) = 0;
            curvature(i) = 0;
        elseif s(i) <= pre_straight_distance + change_distance
            % 向左换道阶段
            ds = s(i) - pre_straight_distance;
            d_prime = polyval(polyder(coeff_left), ds); % 一阶导数
            d_double_prime = polyval(polyder(polyder(coeff_left)), ds); % 二阶导数
            heading_angle(i) = atan(d_prime);
            curvature(i) = d_double_prime / (1 + d_prime^2)^(3/2);
        elseif s(i) <= pre_straight_distance + change_distance + keep_distance
            % 在目标车道保持阶段
            heading_angle(i) = 0;
            curvature(i) = 0;
        elseif s(i) <= pre_straight_distance + change_distance + keep_distance + change_distance
            % 向右换道阶段
            ds = s(i) - (pre_straight_distance + change_distance + keep_distance);
            d_prime = polyval(polyder(coeff_right), ds); % 一阶导数
            d_double_prime = polyval(polyder(polyder(coeff_right)), ds); % 二阶导数
            heading_angle(i) = atan(d_prime);
            curvature(i) = d_double_prime / (1 + d_prime^2)^(3/2);
        else
            % 回到原车道后保持
            heading_angle(i) = 0;
            curvature(i) = 0;
        end
    end
else
    % 定义环形路径的参数
    radius = 30; % 环形路径的半径，单位：米
    num_points = 5000; % 路径上的点数
    theta = linspace(0, 2*pi, num_points); % 从 0 到 2*pi 的角度范围
    
    % 计算环形路径的坐标
    x = radius * cos(theta); % x 坐标
    y = radius * sin(theta); % y 坐标
    s = x;
    d = y;
    
    % 计算航向角
    heading_angle = zeros(size(theta));
    for i = 1:num_points - 1
        dx = x(i + 1) - x(i);
        dy = y(i + 1) - y(i);
        heading_angle(i) = atan2(dy, dx);
    end
    heading_angle(num_points) = heading_angle(num_points - 1); % 处理最后一个点
    
    % 计算曲率
    curvature = ones(size(theta)) / radius; % 环形路径曲率恒定，为 1/radius
end



% 绘制全局路径
figure(20);
subplot(3,1,1);
plot(s, d);
xlabel('纵向距离 (米)');
ylabel('横向位移 (米)');
title('带 50m 前置直线的双移线换道超车全局路径');
grid on;

% 绘制航向角
subplot(3,1,2);
plot(s, rad2deg(heading_angle));
xlabel('纵向距离 (米)');
ylabel('航向角 (度)');
title('航向角变化');
grid on;

% 绘制曲率
subplot(3,1,3);
plot(s, curvature);
xlabel('纵向距离 (米)');
ylabel('曲率');
title('曲率变化');
grid on;