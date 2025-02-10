function [xr, yr, thetar, kappar] = findNearestPoint(path_s, path_d, path_heading_angle, path_curvature, x0, y0)
    % 计算每个点到 (x0, y0) 的欧几里得距离
    distances = sqrt((path_s - x0).^2 + (path_d - y0).^2);

    % 找到最小距离的索引
    [~, minIndex] = min(distances);

    % 找到离 (x0, y0) 最近的点
    xr = path_s(minIndex);
    yr = path_d(minIndex);
    thetar = path_heading_angle(minIndex);
    kappar = path_curvature(minIndex);
end