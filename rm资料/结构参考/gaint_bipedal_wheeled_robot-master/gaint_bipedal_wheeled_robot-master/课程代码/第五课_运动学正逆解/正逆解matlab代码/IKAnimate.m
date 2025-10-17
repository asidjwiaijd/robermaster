function IKAnimate()
    L1 = 150;
    L2 = 250;
    L3 = 250;
    L4 = 150;
    L5 = 108;

    %轨迹生成2
    t = linspace(-10, 118, 128);
    offset = 240;
    amp = 40;
    period = 128;
    X_traj = t;
    Y_traj = offset + amp * sin((2*pi/period)*t);
    
    figure; hold on;  grid on;
    % axis equal;
    title('简化五连杆机构图');
    xlabel('X 轴 (cm)');
    ylabel('Y 轴 (cm)');
    xlim([-180 300]);   % X 轴范围从 -1 到 1
    ylim([-20 300]);   % Y 轴范围从 -1 到 1 
    set(gca, 'YDir', 'reverse')

    BTraj = [];
    hL1 = plot([0, 0], [0, 0], 'k', 'LineWidth', 2);
    hL2 = plot([0, 0], [0, 0], 'k', 'LineWidth', 2);
    hL3 = plot([0, 0], [0, 0], 'k', 'LineWidth', 2);
    hL4 = plot([0, 0], [0, 0], 'k', 'LineWidth', 2);
    hL5 = plot([0, 0], [0, 0], 'k', 'LineWidth', 2);

    
    hAText = text(0, 0, 'A', 'FontSize', 12, 'Color', 'k', 'FontWeight', 'bold');
    hBText = text(0, 0, 'B(0,0)', 'FontSize', 12, 'Color', 'k', 'FontWeight', 'bold');
    hCText = text(0, 0, 'C', 'FontSize', 12, 'Color', 'k', 'FontWeight', 'bold');
    hDText = text(0, 0, 'D', 'FontSize', 12, 'Color', 'k', 'FontWeight', 'bold');
    hOText = text(0, 0, 'O', 'FontSize', 12, 'Color', 'k', 'FontWeight', 'bold');
    hAlphaBetaText = text(-150,220,'\alpha:0° \newline\beta:0°','FontSize', 18, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top','FontWeight', 'bold');
    hIKText = text(-150,200,'逆解得到的角度','FontSize', 10, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
    hTrace = plot(NaN, NaN, 'r.', 'MarkerSize', 5);  % 轨迹点

    while true
        for i = 1:length(X_traj)
            X = X_traj(i);
            Y = Y_traj(i);
            % 调用逆解函数
            IKResult = inverseKinematics(X, Y);
            anglePairs = {
                IKResult.alpha1, IKResult.beta1;
                IKResult.alpha1, IKResult.beta2;
                IKResult.alpha2, IKResult.beta1;
                IKResult.alpha2, IKResult.beta2
            };
            selected.alpha = IKResult.alpha;
            selected.beta = IKResult.beta;

            % 计算关键点坐标
            O = [0, 0];                             % 起点 O
            A = [L1 * cos(selected.alpha), L1 * sin(selected.alpha)];   % 第一连杆末端点 P1
            B = [X, Y];                            % 末端点（输入）
            % 另一组连杆的起点
            D = [L5, 0];
            C = [D(1) + L4 * cos(selected.beta), D(2) + L4 * sin(selected.beta)];

            % 保存轨迹
            BTraj = [BTraj; B];

            % 更新连杆
            set(hL3, 'XData', [C(1), B(1)], 'YData', [C(2), B(2)]);
            set(hL4, 'XData', [D(1), C(1)], 'YData', [D(2), C(2)]);
            set(hL5, 'XData', [O(1), D(1)], 'YData', [O(2), D(2)]);
            set(hL1, 'XData', [O(1), A(1)], 'YData', [O(2), A(2)]);
            set(hL2, 'XData', [A(1), B(1)], 'YData', [A(2), B(2)]);

            set(hAText,'Position', [A(1)-20,A(2)]);
            set(hBText,'Position', [B(1),B(2)+10],'String',sprintf('B(%d,%d)',round(B(1)),round(B(2))));
            set(hCText,'Position', [C(1)+20,C(2)+5]);
            set(hDText,'Position', [D(1),D(2)-10]);
            set(hOText,'Position', [O(1),O(2)-10]);

            set(hAlphaBetaText,'String',sprintf('α:%.2f° \nβ:%.2f° ', rad2deg(selected.alpha), rad2deg(selected.beta)))
    
            % 更新轨迹
            set(hTrace, 'XData', BTraj(:,1), 'YData', BTraj(:,2));
            drawnow;
            pause(0.03);
        end
    end
end