%此代码用来求解VMC.slx中的y'max参数
clc;
clear;
l1=0.1;
l2=0.16;
l3=0.32;
Angle=[150:240];
angle=Angle.*pi/180;
syms x
y=sqrt(l3^2-(l1-l2*cos(x))^2)-l2*sin(x);
y1=diff(y,x);
Y1=eval(subs(y1,x,angle));
[Y1max,p]=max(Y1);