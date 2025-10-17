function result = forwardKinematics(alpha,beta)
    L1 = 150;
    L2 = 250;
    L3 = 250;
    L4 = 150;
    L5 = 108;

    alpha = deg2rad(alpha);
    beta = deg2rad(beta);
    
    % A C的坐标计算
    xa = L1 * cos(alpha);
    ya = L1 * sin(alpha);
    xc = L5 + L4 * cos(beta);
    yc = L4 * sin(beta);

    % 三角几何项计算
    a = 2 * (xa - xc) * L2;
    b = 2 * (ya - yc) * L2;
    LAC = sqrt((xa - xc)^2 + (ya - yc)^2);
    c = L3^2 - L2^2 - LAC^2;

    % θ1求解
    theta1_1 =  2 * atan2(b + sqrt(a^2 + b^2 - c^2), a + c);
    theta1_2 =  2 * atan2(b - sqrt(a^2 + b^2 - c^2), a + c);
    % 解的选择（限制范围）
    theta1_1 = mod(theta1_1, 2*pi);
    theta1_2 = mod(theta1_2, 2*pi);
    if theta1_1 < pi/2
        theta1 = theta1_1;
    else
        theta1 = theta1_2;
    end

    x = L1 * cos(alpha) + L2 * cos(theta1);
    y = L1 * sin(alpha) + L2 * sin(theta1);

    x_1 = L1 * cos(alpha) + L2 * cos(theta1_1);
    y_1 = L1 * sin(alpha) + L2 * sin(theta1_1);

    x_2 = L1 * cos(alpha) + L2 * cos(theta1_2);
    y_2 = L1 * sin(alpha) + L2 * sin(theta1_2);


    % 机器人可行解
    result.theta1 = theta1;
    result.x = x;
    result.y = y;

    result.theta1_1 = theta1_1;
    result.x_1 = x_1;
    result.y_1 = y_1;
    result.theta1_2 = theta1_2;
    result.x_2 = x_2;
    result.y_2 = y_2;
    
    result.xa = xa;
    result.ya = ya;
    result.xc = xc;
    result.yc = yc;