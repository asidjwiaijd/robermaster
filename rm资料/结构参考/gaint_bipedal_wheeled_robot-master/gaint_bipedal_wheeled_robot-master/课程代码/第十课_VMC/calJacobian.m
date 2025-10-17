syms l1 l2 l3 l4 l5
syms alpha(t) beta(t) theta1(t) theta2(t)
syms theta1_dot theta2_dot 

x_A = l1*cos(alpha); y_A = l1*sin(alpha); % A点坐标
x = x_A+l2*cos(theta1); y = y_A+l2*sin(theta1);% B点坐标
x_C = l5+l4*cos(beta); y_C = l4*sin(beta); % C点坐标


x_dot_A = diff(x_A,t); y_dot_A = diff(y_A,t);% A点坐标导数
x_dot_C = diff(x_C,t); y_dot_C = diff(y_C,t);% C点坐标导数


eq1 = x_dot_A - l2*theta1_dot*sin(theta1) == x_dot_C - l3*theta2_dot*sin(theta2); % 关于A和C点的中间变量式
eq2 = y_dot_A + l2*theta1_dot*cos(theta1) == y_dot_C + l3*theta2_dot*cos(theta2);

S = solve([eq1,eq2],[theta1_dot,theta2_dot]);    
theta1_dot_simplified = simplify(S.theta1_dot);



%theta1_dot_simplified =
%-(l1*sin(alpha(t) - theta2(t))*diff(alpha(t), t) - l4*sin(beta(t) - theta2(t))*diff(beta(t), t))/(l2*sin(theta1(t) - theta2(t)))

v_x = subs(x_dot_B, diff(theta1,t), theta1_dot_simplified);
v_x = subs(x_dot_B, [diff(alpha,t), diff(beta,t)], [alpha_dot, beta_dot]);

v_y = subs(y_dot_B, diff(theta1,t), theta1_dot_simplified);
v_y = subs(y_dot_B, [diff(alpha,t), diff(beta,t)], [alpha_dot, beta_dot]);


v = [v_x; v_y];
q_dot = [alpha_dot; beta_dot];

v = simplify(collect(v, q_dot));
J = simplify(jacobian(v, q_dot))
