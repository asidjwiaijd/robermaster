function IKPlotter(X, Y)
    % 连杆长度参数
    L1 = 150;
    L2 = 250;
    L3 = 250;
    L4 = 150;
    L5 = 108;

    IKResult = inverseKinematics(X, Y);
    anglePairs = {
        IKResult.alpha1, IKResult.beta1;
        IKResult.alpha1, IKResult.beta2;
        IKResult.alpha2, IKResult.beta1;
        IKResult.alpha2, IKResult.beta2
    };
    selected.alpha = IKResult.alpha;
    selected.beta = IKResult.beta;
    disp(selected);

%-------------计算四个解-----------------------
    figure;
    for i = 1:4
        alpha = anglePairs{i,1};
        beta  = anglePairs{i,2};

        % 点坐标计算
        O = [0, 0];                           % 原点
        A = O + L1 * [cos(alpha), sin(alpha)]; % A点
        B = [X, Y];                           % 末端目标点
        D = [L5, 0];                      % 另一个基点
        C = D + L4 * [cos(beta), sin(beta)]; % D点

        subplot(2, 2, i); hold on; axis equal; grid on;
        xlim([-180 280]);   % X 轴范围从 -1 到 1
        ylim([-20 270]);   % Y 轴范围从 -1 到 1 
        set(gca, 'YDir', 'reverse');
        title(sprintf('\\alpha = %.1f°, \\beta = %.1f°', rad2deg(alpha), rad2deg(beta)));

        % 绘制连杆
        plot([O(1), A(1)], [O(2), A(2)], 'k-', 'LineWidth', 2); % L1
        plot([A(1), B(1)], [A(2), B(2)], 'k-', 'LineWidth', 2); % L2
        plot([D(1), C(1)], [D(2), C(2)], 'k-', 'LineWidth', 2); % L4
        plot([C(1), B(1)], [C(2), B(2)], 'k-', 'LineWidth', 2); % L3
        plot([O(1), D(1)], [O(2), D(2)], 'k-', 'LineWidth', 2); % L5

        % 标注点名
        % text(O(1)-20, O(2), 'O', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment','right');
        % text(A(1)-10, A(2), 'A', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment','right');
        % text(B(1), B(2)-30, 'B', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment','right');
        % text(C(1)+20, C(2), 'C', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment','right');
        % text(D(1)+20, D(2), 'D', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment','right');
    end

%-------------计算四个解-----------------------


% %-------------计算唯一解-----------------------
%     % 计算关键点坐标
%     O = [0, 0];                             % 起点 O
%     A = [L1 * cos(selected.alpha), L1 * sin(selected.alpha)];   % 第一连杆末端点 P1
%     B = [X, Y];                            % 末端点（输入）
% 
%     % 另一组连杆的起点
%     D = [L5, 0];
%     C = [D(1) + L4 * cos(selected.beta), D(2) + L4 * sin(selected.beta)];
%     % disp(C);
% 
%     % 绘图
%     figure; hold on; axis equal; grid on;
%     title('简化五连杆机构图');
%     xlabel('X 轴 (cm)');
%     ylabel('Y 轴 (cm)');
%     xlim([-180 300]);   % X 轴范围从 -1 到 1
%     ylim([-20 300]);   % Y 轴范围从 -1 到 1 
%     set(gca, 'YDir', 'reverse')
% 
%     % 连杆绘制
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
%     text(O(1)-15, O(2)+10, sprintf('\\alpha = %.1f^\\circ', rad2deg(selected.alpha)), ...
%     'FontSize', 10, 'Color', 'k');
% 
% text(D(1)+30, D(2)+5, sprintf('\\beta = %.1f^\\circ', rad2deg(selected.beta)), ...
%     'FontSize', 10, 'Color', 'k');
%-------------计算唯一解-----------------------