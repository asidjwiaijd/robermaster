clc
clear
R=zeros(89,89);%相关系数矩阵
H=zeros(89,89);%最高高度矩阵
L=zeros(89,89);%最低高度矩阵
Y=zeros(89,89);%高度差矩阵
B=zeros(89,89);%l2杆长矩阵
C=zeros(89,89);%l3杆长矩阵
for Anglemin=91:179 %最小角度限制在90-180之间
    for Anglemax=271:359 %最大角度限制在270-360之间
        if(Anglemax>=(Anglemin+180)) %最大角度<最小角度+180
            break;
        end
        Radmin=2*pi*Anglemin/360;
        Radmax=2*pi*Anglemax/360;
        Angle=[Anglemin:1:Anglemax];
        Rad=2*pi*Angle/360;
        a=0.1; %设定l1杆的长度
        b=(a/cos(Radmax)+a/cos(Radmin))/2;
        c=(a/cos(Radmax)-a/cos(Radmin))/2;
        y=sqrt(c^2-(a-b*cos(Rad)).^2)-b*sin(Rad);%角度与高度的关系式
        R((Anglemin-90),(Anglemax-270))=min(min(corrcoef(Angle,y)));%获取角度与高度之间的相关系数
        L((Anglemin-90),(Anglemax-270))=-a*tan(Radmin);%获取最低高度
        H((Anglemin-90),(Anglemax-270))=-a*tan(Radmax);%获取最高高度
        Y((Anglemin-90),(Anglemax-270))=a*(tan(Radmin)-tan(Radmax));%获取高度差
        B((Anglemin-90),(Anglemax-270))=b;%获取l2杆长
        C((Anglemin-90),(Anglemax-270))=c;%获取l3杆长
    end
end
AngleSet=zeros(2,89);%存放筛选后角度值的矩阵
k=0;
for i=1:89
    for j=1:89
        if(R(i,j)>=0.98)%以相关系数作为筛选条件
            if((0.4<=H(i,j))&&(H(i,j)<=0.5))%以最高高度作为筛选条件
                  if((0.1<=L(i,j))&&(L(i,j)<=0.2))%以最低高度作为筛选条件
                       if((0.3<=Y(i,j))&&(Y(i,j)<=0.4))%以高度差作为筛选条件
                        k=k+1;
                        AngleSet(1,k)=i+90;%筛选后的最小角度存入矩阵
                        AngleSet(2,k)=j+270;%筛选后的最大角度存入矩阵
                       end
                  end
            end
        end
    end
end
Final=zeros(8,k);%存放筛选后的角度，以及与之对应的相关数据
for p=1:k
    Final(1,p)=AngleSet(1,p);%最小角度
    Final(2,p)=AngleSet(2,p);%最大角度
    Final(3,p)=R((Final(1,p)-90),(Final(2,p)-270));%相关系数
    Final(4,p)=H((Final(1,p)-90),(Final(2,p)-270));%最高高度
    Final(5,p)=L((Final(1,p)-90),(Final(2,p)-270));%最低高度
    Final(6,p)=Y((Final(1,p)-90),(Final(2,p)-270));%高度差
    Final(7,p)=B((Final(1,p)-90),(Final(2,p)-270));%l2杆长
    Final(8,p)=C((Final(1,p)-90),(Final(2,p)-270));%l3杆长
end
%将所有矩阵输出到excel文件中
xlswrite('E:\设计尺寸总表',R,'相关系数');
xlswrite('E:\设计尺寸总表',H,'最高高度');
xlswrite('E:\设计尺寸总表',L,'最低高度');
xlswrite('E:\设计尺寸总表',Y,'高度差');
xlswrite('E:\设计尺寸总表',B,'l2杆长');
xlswrite('E:\设计尺寸总表',C,'l3杆长');
xlswrite('E:\设计尺寸总表',Final,'筛选后数据');