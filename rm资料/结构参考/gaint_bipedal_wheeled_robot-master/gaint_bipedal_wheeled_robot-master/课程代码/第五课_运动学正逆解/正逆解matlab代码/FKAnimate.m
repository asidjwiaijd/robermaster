function FKAnimate()
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
    title('正解出来的足端轨迹');
    xlabel('X 轴 (cm)');
    ylabel('Y 轴 (cm)');
    xlim([-180 300]);   % X 轴范围从 -1 到 1
    ylim([-20 300]);   % Y 轴范围从 -1 到 1 
    set(gca, 'YDir', 'reverse')

    htoFKText = text(-150,140,'放入正解的角度','FontSize', 10, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
    hAlphaBetaText = text(-150,150,'\alpha:0° \newline\beta:0°','FontSize', 18, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top','FontWeight', 'bold');
    
    hafterFKText = text(-150,220,'正解出来的坐标','FontSize', 10, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
    hBCoordText = text(-150,230,'B(0,0)','FontSize', 18, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top','FontWeight', 'bold', 'Color', 'r');

    hTrace = plot(NaN, NaN, 'r.', 'MarkerSize', 5);  % 轨迹点
    hBPoint = plot(NaN, NaN, 'k.', 'MarkerSize', 30); % 运动点
    BTraj = [];

    while true
        for i = 1:length(X_traj)
            X = X_traj(i);
            Y = Y_traj(i);
            % 调用逆解函数,根据B点轨迹点求出逆解
            IKResult = inverseKinematics(X, Y);
            B = [X,Y];
            
            % 将逆解出来的α和β放进正解，求出B点坐标
            FKResult = forwardKinematics(rad2deg(IKResult.alpha),rad2deg(IKResult.beta));
            FK_B = [FKResult.x,FKResult.y];

            % 保存轨迹
            BTraj = [BTraj; B];

            set(hAlphaBetaText,'String',sprintf('α:%.2f° \nβ:%.2f° ', rad2deg(IKResult.alpha), rad2deg(IKResult.beta)));
            set(hBCoordText,'String',sprintf('B(%d,%d)', round(FK_B(1)),round(FK_B(2))));

            % 更新轨迹
            set(hTrace, 'XData', BTraj(:,1), 'YData', BTraj(:,2));
            set(hBPoint, 'XData', FK_B(1), 'YData', FK_B(2));

            drawnow;
            pause(0.03);
        end
    end