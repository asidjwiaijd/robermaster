%给定最大、最小角度，作出位移-角度，及其一阶导、二阶导图像
clc
clear
% 角度初始化
Anglemax=282;%给定最大角度
Radmax=2*pi*Anglemax/360;
Anglemin=129;%给定最小角度
Radmin=2*pi*Anglemin/360;
Angle=[Anglemin:1:Anglemax];
Rad=2*pi*Angle/360;
% 杆长初始化
a=0.1;%给定机架长度
b=(a/cos(Radmax)+a/cos(Radmin))/2;
c=(a/cos(Radmax)-a/cos(Radmin))/2;
% 列出位移与角度的函数，并求其一阶、二阶导数
syms x 
f=sqrt(c^2-(a-b*cos(x))^2)-b*sin(x);
f1=diff(f);
f2=diff(f,2);
% 将角度数据代入求出函数值
y=subs(f,Rad);
y1=subs(f1,Rad);
y2=subs(f2,Rad);
% 作图
figure(1)%位移-角度
plot(Angle,y)
xlabel('关节电机角度/°')
ylabel('机体高度/m')
title('关节电机角度—机体高度')
figure(2)%一阶导
plot(Angle,y1)
figure(3)%二阶导
plot(Angle,y2)