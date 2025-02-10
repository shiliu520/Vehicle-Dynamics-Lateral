function [e_d, e_psi, kappa] = error_calc_preview(x, y, psi, L, xr, yr, thetar, curvature)
    % 计算预瞄点的全局坐标
    x_pre = x + L * cos(psi);
    y_pre = y + L * sin(psi);
    
    %% 射线法
    % 找到距离最近的点的索引
    min_distance = inf;
    index = NaN;
    % 射线方向向量
    ray_angle = psi + pi/2;
    dx = cos(ray_angle);
    dy = sin(ray_angle);
    
    for i = 1:length(xr) - 1
        % 路径线段的两个端点
        x1 = xr(i);
        y1 = yr(i);
        x2 = xr(i + 1);
        y2 = yr(i + 1);
        
        % 计算射线与线段的交点
        den = (dx * (y2 - y1) - dy * (x2 - x1));
        if den ~= 0
            t = ((x1 - x_pre) * (y2 - y1) - (y1 - y_pre) * (x2 - x1)) / den;
            u = ((x1 - x_pre) * dy - (y1 - y_pre) * dx) / den;
            if t >= 0 && u >= 0 && u <= 1
                intersect_x = x_pre + t * dx;
                intersect_y = y_pre + t * dy;
                distance = sqrt((intersect_x - x_pre)^2 + (intersect_y - y_pre)^2);
                if distance < min_distance
                    min_distance = distance;
                    index = i + u; % 交点在线段上的相对位置
                end
            end
        end
    end
    % 根据交点相对位置获取相关信息
    x_A = interp1(1:length(xr), xr, index);
    y_A = interp1(1:length(yr), yr, index);
    kappa = interp1(1:length(curvature), curvature, index);
    % 插值获取点 A 对应的全局路径航向角
    theta_rA = interp1(1:length(thetar), thetar, index);
    
    %% 最近点代替
    % 计算预瞄点到全局路径上各点的距离
    distances = sqrt((xr - x_pre).^2 + (yr - y_pre).^2);
    [~, index] = min(distances);
    % 获取最近点 A 的全局坐标
    x_A = xr(index);
    y_A = yr(index);
    kappa = curvature(index);
    % 获取点 A 对应的全局路径航向角
    theta_rA = thetar(index);
    
    %%
    % 将点 A 的坐标转换到车辆局部坐标系下
    x_A_local = (x_A - x) * cos(psi) + (y_A - y) * sin(psi);
    y_A_local = -(x_A - x) * sin(psi) + (y_A - y) * cos(psi);
    
    % 横向误差 e_d 即为点 A 在车辆局部坐标系下的 y 坐标
    e_d = y_A_local;
    
    % 计算航向误差
    e_psi = theta_rA - psi;
    
    % 将航向误差限制在 [-pi, pi] 范围内
    e_psi = atan2(sin(e_psi), cos(e_psi));
end