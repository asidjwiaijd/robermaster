function result = inverseKinematics(X, Y)
    L1 = 150;
    L2 = 250;
    L3 = 250;
    L4 = 150;
    L5 = 108;
    
    % 三角几何项计算
    a = 2 * X * L1;
    b = 2 * Y * L1;
    c = X^2 + Y^2 + L1^2 - L2^2;
    
    d = 2 * L4 * (X - L5);
    e = 2 * L4 * Y;
    f = (X - L5)^2 + L4^2 + Y^2 - L3^2;
    
    % α 与 β 的两个解
    alpha1 = 2 * atan2(b + sqrt(a^2 + b^2 - c^2), a + c);
    alpha2 = 2 * atan2(b - sqrt(a^2 + b^2 - c^2), a + c);
    
    beta1 = 2 * atan2(e + sqrt(d^2 + e^2 - f^2), d + f);
    beta2 = 2 * atan2(e - sqrt(d^2 + e^2 - f^2), d + f);
    
    % 解的选择逻辑（限制范围）
    alpha1 = mod(alpha1, 2*pi);
    alpha2 = mod(alpha2, 2*pi);

    beta1 = mod(beta1, 2*pi);
    beta2 = mod(beta2, 2*pi);
    
    if alpha1 >= pi/4
        alpha = alpha1;
    else
        alpha = alpha2;
    end
    
    if beta1 <= pi/4
        beta = beta1;
    else
        beta = beta2;
    end

    result.alpha = alpha;
    result.beta = beta;
    result.alpha1 = alpha1;
    result.beta1 = beta1;
    result.alpha2 = alpha2;
    result.beta2 = beta2;




%-------------计算唯一解-----------------------
%     % 计算关键点坐标
%     O = [0, 0];                             % 起点 O
%     A = [L1 * cos(alpha), L1 * sin(alpha)];   % 第一连杆末端点 P1
%     B = [X, Y];                            % 末端点（输入）
% 
%     % 另一组连杆的起点
%     D = [L5, 0];
%     C = [D(1) + L4 * cos(beta), D(2) + L4 * sin(beta)];
%     % disp(C);
%     % 绘图
%     figure; hold on; axis equal; grid on;
%     title('简化五连杆机构图');
%     xlabel('X 轴 (m)');
%     ylabel('Y 轴 (m)');
%     set(gca, 'YDir', 'reverse')
% 
%     % 连杆绘制
%     plot([O(1), A(1)], [O(2), A(2)], 'k-', 'LineWidth', 2); % L1
%     plot([A(1), B(1)], [A(2), B(2)], 'k-', 'LineWidth', 2); % L2
%     plot([D(1), C(1)], [D(2), C(2)], 'k-', 'LineWidth', 2); % L4
%     plot([C(1), B(1)], [C(2), B(2)], 'k-', 'LineWidth', 2); % L3
%     plot([O(1), D(1)], [O(2), D(2)], 'k-', 'LineWidth', 2); % L5
% 
%     text(O(1)-15, O(2)+10, sprintf('\\alpha = %.1f^\\circ', rad2deg(alpha)), ...
%     'FontSize', 10, 'Color', 'k');
% 
% text(D(1)+30, D(2)+5, sprintf('\\beta = %.1f^\\circ', rad2deg(beta)), ...
%     'FontSize', 10, 'Color', 'k');
%-------------计算唯一解-----------------------

%-------------计算四个解-----------------------

    % angle_pairs = {
    %     alpha1, beta1;
    %     alpha1, beta2;
    %     alpha2, beta1;
    %     alpha2, beta2
    % };
    % figure;
    % for i = 1:4
    %     alpha = angle_pairs{i,1};
    %     beta  = angle_pairs{i,2};
    % 
    %     % 点坐标计算
    %     O = [0, 0];                           % 原点
    %     A = O + L1 * [cos(alpha), sin(alpha)]; % A点
    %     B = [X, Y];                           % 末端目标点
    %     D = [L5, 0];                      % 另一个基点
    %     C = D + L4 * [cos(beta), sin(beta)]; % D点
    %     % C 是 D→B 方向，未单独命名，因为直接连线
    % 
    %     subplot(2, 2, i); hold on; axis equal; grid on;
    %     set(gca, 'YDir', 'reverse');
    %     title(sprintf('\\alpha = %.1f°, \\beta = %.1f°', rad2deg(alpha), rad2deg(beta)));
    % 
    %     % 绘制连杆
    %     plot([O(1), A(1)], [O(2), A(2)], 'k-', 'LineWidth', 2); % L1
    %     plot([A(1), B(1)], [A(2), B(2)], 'k-', 'LineWidth', 2); % L2
    %     plot([D(1), C(1)], [D(2), C(2)], 'k-', 'LineWidth', 2); % L4
    %     plot([C(1), B(1)], [C(2), B(2)], 'k-', 'LineWidth', 2); % L3
    %     plot([O(1), D(1)], [O(2), D(2)], 'k-', 'LineWidth', 2); % L5
    % 
    %     % 标注点名
    %     text(O(1)-20, O(2), 'O', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment','right');
    %     text(A(1)-10, A(2), 'A', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment','right');
    %     text(B(1), B(2)-30, 'B', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment','right');
    %     text(C(1)+20, C(2), 'C', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment','right');
    %     text(D(1)+20, D(2), 'D', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment','right');
    % 
    %     % % 标注角度
    %     % text(O(1)+0.01, O(2)+0.01, sprintf('\\alpha = %.1f°', rad2deg(alpha)), 'FontSize', 9);
    %     % text(C(1)+0.01, C(2)+0.01, sprintf('\\beta = %.1f°', rad2deg(beta)), 'FontSize', 9);
    % 
    %     % % 添加弧线和角度标注（L1 和 L5 之间的角度，标为 alpha）
    %     % angleRadius = 0.1; % 弧线半径
    %     % theta = linspace(alpha - pi/10, alpha + pi/10, 100); % 计算弧线
    %     % arcX = O(1) + angleRadius * cos(theta); % 弧线 X 坐标
    %     % arcY = O(2) + angleRadius * sin(theta); % 弧线 Y 坐标
    %     % plot(arcX, arcY, 'k--', 'LineWidth', 1); % 弧线
    %     % 
    %     % % 标注角度名称
    %     % text(O(1) + angleRadius * cos(alpha), O(2) + angleRadius * sin(alpha), 'alpha', 'FontSize', 9, 'FontWeight', 'bold');
    % end

%-------------计算四个解-----------------------
end


