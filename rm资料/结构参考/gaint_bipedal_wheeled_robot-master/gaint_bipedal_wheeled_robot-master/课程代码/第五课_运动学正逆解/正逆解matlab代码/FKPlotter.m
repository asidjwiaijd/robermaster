function FKPlotter(alpha,beta)

    L1 = 150;
    L2 = 250;
    L3 = 250;
    L4 = 150;
    L5 = 108;

    FKResult = forwardKinematics(alpha,beta);

    % 计算关键点坐标
    O = [0, 0];     
    A = [FKResult.xa, FKResult.ya];
    B = [FKResult.x, FKResult.y];
    C = [FKResult.xc, FKResult.yc];
    D = [L5, 0];

 % ------------------绘制所有解------------------------
    figure('Name', '运动学正解出的两个坐标点');

    % 点坐标计算
    O = [0, 0];                           % 原点
    A = [FKResult.xa,FKResult.ya]; % A点
    D = [L5, 0];                      % D点
    C = [FKResult.xc,FKResult.yc]; % C点
    B_1 = [FKResult.x_1,FKResult.y_1];
    B_2 = [FKResult.x_2,FKResult.y_2];

    % 图1
    subplot(1,2,1);
    hold on;axis equal; grid on;
    xlim([-150 320]);   % X 轴范围从 -1 到 1
    ylim([-100 320]);   % Y 轴范围从 -1 到 1 
    set(gca, 'YDir', 'reverse');

    title(sprintf('B(%d,%d)', round(B_1(1)), round(B_1(2))));

    plot([O(1), A(1)], [O(2), A(2)], 'k-', 'LineWidth', 2); % L1
    plot([A(1), B_1(1)], [A(2), B_1(2)], 'k-', 'LineWidth', 2); % L2
    plot([D(1), C(1)], [D(2), C(2)], 'k-', 'LineWidth', 2); % L4
    plot([C(1), B_1(1)], [C(2), B_1(2)], 'k-', 'LineWidth', 2); % L3
    plot([O(1), D(1)], [O(2), D(2)], 'k-', 'LineWidth', 2); % L5


    % 图2
    subplot(1,2,2);
    hold on;axis equal; grid on;
    xlim([-150 320]);   % X 轴范围从 -1 到 1
    ylim([-100 320]);   % Y 轴范围从 -1 到 1 
    set(gca, 'YDir', 'reverse');

    title(sprintf('B(%d,%d)', round(B_2(1)), round(B_2(2))));

    plot([O(1), A(1)], [O(2), A(2)], 'k-', 'LineWidth', 2); % L1
    plot([A(1), B_2(1)], [A(2), B_2(2)], 'k-', 'LineWidth', 2); % L2
    plot([D(1), C(1)], [D(2), C(2)], 'k-', 'LineWidth', 2); % L4
    plot([C(1), B_2(1)], [C(2), B_2(2)], 'k-', 'LineWidth', 2); % L3
    plot([O(1), D(1)], [O(2), D(2)], 'k-', 'LineWidth', 2); % L5
% ------------------绘制所有解------------------------


 % % ------------------绘制唯一解------------------------
 %    % 绘图
 %    figure('Name', '运动学正解出的合理坐标点');
 %    hold on; axis equal; grid on;
 %    title('简化五连杆机构图');
 %    xlabel('X 轴 (cm)');
 %    ylabel('Y 轴 (cm)');
 %    set(gca, 'YDir', 'reverse')
 % 
 %    % 标注点名
 %    text(O(1)-20, O(2), 'O', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment','right');
 %    text(A(1)-10, A(2), 'A', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment','right');
 %    text(B(1)-20, B(2)+10, 'B', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment','right');
 %    text(C(1)+20, C(2), 'C', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment','right');
 %    text(D(1)+20, D(2), 'D', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment','right');
 % 
 %    % 连杆绘制
 %    plot([O(1), A(1)], [O(2), A(2)], 'k-', 'LineWidth', 2); % L1
 %    plot([A(1), B(1)], [A(2), B(2)], 'k-', 'LineWidth', 2); % L2
 %    plot([D(1), C(1)], [D(2), C(2)], 'k-', 'LineWidth', 2); % L4
 %    plot([C(1), B(1)], [C(2), B(2)], 'k-', 'LineWidth', 2); % L3
 %    plot([O(1), D(1)], [O(2), D(2)], 'k-', 'LineWidth', 2); % L5
 % 
 %    text(B(1)-15, B(2)+10, sprintf('(%.1f,%.1f)', FKResult.x,FKResult.y), ...
 %    'FontSize', 10, 'Color', 'k','FontWeight', 'bold');
 % % ------------------绘制唯一解------------------------
