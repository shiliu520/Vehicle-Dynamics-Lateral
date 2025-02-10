function [x_preview, y_preview] = find_preview_point(x_, y_, psi_, L)
    % 判断每个 psi_ 是否接近 pi/2 或 -pi/2
    special_case = abs(abs(psi_) - pi/2) < 1e-10;

    % 处理特殊情况（垂直向上或向下）
    vertical_up = psi_ > 0 & special_case;
    vertical_down = psi_ < 0 & special_case;

    % 计算特殊情况下的预瞄点坐标
    x_preview_special = x_;
    y_preview_special_up = y_ + L * vertical_up;
    y_preview_special_down = y_ - L * vertical_down;
    y_preview_special = y_preview_special_up + y_preview_special_down;

    % 处理一般情况
    general_case = ~special_case;
    cos_psi = cos(psi_);
    sin_psi = sin(psi_);
    delta_x_general = L * cos_psi;
    delta_y_general = L * sin_psi;
    x_preview_general = x_ + delta_x_general .* general_case;
    y_preview_general = y_ + delta_y_general .* general_case;

    % 合并特殊情况和一般情况的结果
    x_preview = x_preview_special .* special_case + x_preview_general .* general_case;
    y_preview = y_preview_special .* special_case + y_preview_general .* general_case;
end