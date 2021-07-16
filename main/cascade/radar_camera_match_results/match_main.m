%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%功能：实现联合标定
%输入：
% data:标定数据矩阵 N×5矩阵，N为样本数（不超过20），对于每个样本，前三个数表示雷达坐标[x,y,z]（float），后两个数表示对应的图像坐标点[xp,yp]（uint16）;
% A:内参矩阵 3×3矩阵（double float）
% B:外参矩阵 3×4矩阵（float）
%输出：
% H:坐标转移矩阵 3×4矩阵（double float）
%相关函数：
%psolution(alpha,fk,Jfk,Dk)：psolution(lambda,f,Jf,D)求p的最小二乘解
%dphi = dphisolution(lambda,f,Jf,D)：求dphi/dlambda
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%初始化
clear all;
close all;
x=[sym('thetax');sym('thetay');sym('thetaz');sym('tx');sym('ty');sym('tz')]; %定义函数变量 x，共12个变量，x(1)~x(12)（double float）
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 仿真数据生成
% N=input('请输入样本数N的值:\n');
% N=25;
% for i=1:N
%     fprintf('请输入第%d个样本数据[xr,yr,zr,xp,yp]:',i)
%     data(i,:)=input('\n');
% end
%转移矩阵设定
% A=[800,0,200;0,800,200;0,0,1];     %内参矩阵
% R=[1,0,0;0,1,0;0,0,1];             %旋转矩阵
% t=[1;1;1];                         %平移矩阵
% H=A*[R,t];                         %单应矩阵
%数据生成
% data=zeros(N,5);
% data=10*rand(N,5);
% for i=1:N
%     xy=H*[data(i,1:3),1]';
%     data(i,4)=xy(1)/xy(3);
%     data(i,4)=data(i,4)*(1+0.01*randn(1));
%     data(i,5)=xy(2)/xy(3);
%     data(i,5)=data(i,5)*(1+0.01*randn(1));
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%实验数据读取（根据需求，与上面的仿真数据生成两者取1）
% load('data_calibration.mat');%读取用于标定的数据
% data=data_calibration;%标定数据矩阵赋值（输入，double float）
% N=size(data,1);%读取数据长度（uint8）
% load('A.mat');
% A=[680.52,0,279.52;0,683.47,266.69;0,0,1];%内参矩阵赋值（输入，double float）
Rx=[1,0,0;0,cos(x(1)),sin(x(1));0,-sin(x(1)),cos(x(1))];
Ry=[cos(x(2)),0,-sin(x(2));0,1,0;sin(x(2)),0,cos(x(2))];
Rz=[cos(x(3)),-sin(x(3)),0;sin(x(3)),cos(x(3)),0;0,0,1];
B=[Rx*Ry*Rz,[x(4);x(5);x(6)]];
% load('B_initial.mat');%读取外参矩阵
% B=B_initial;%外参矩阵复制（输入，double float）
% H_initial=A*B;%计算初始坐标转移矩阵（double float）
% save H_initial H_initial;%保存初始坐标转移矩阵
% H=reshape(H_initial',12,1);%矩阵重构，变为12×1（double float）

% xk=H;%变量初值赋值（double float）
% A = [964.0495477017739,0,0;0,964.7504668505122,0;613.3463725824778,377.1040664564536,1]; % intrinsic of our logi camera 1280*720
A = [1445.63012662922 0	0; 0 1445.10705595391 0; 920.902144019657 564.645986732188 1];% intrinsic of our logi camera 1920*1080

xk=[-pi/2;0;0;0;-0.07;-0.04];
A=A';

data_xyz=xlsread('20210626match_radar_results.xlsx');
load 20210626match_image_points.mat
data(:, 1:3)=data_xyz(:, 2:4);
save calibdata data


N=size(data,1);%读取数据长度（uint8）

% 雷达向前为y，向右为x，向上为z
% 

H=A*B;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%优化函数构造
f=[];
for i=1:N%样本优化函数构造
%     f(i,1)=sqrt(((data(i,1)*x(1)+data(i,2)*x(2)+data(i,3)*x(3)+x(4))/(data(i,1)*x(9)+data(i,2)*x(10)+data(i,3)*x(11)+x(12))-data(i,4))^2+((data(i,1)*x(5)+data(i,2)*x(6)+data(i,3)*x(7)+x(8))/(data(i,1)*x(9)+data(i,2)*x(10)+data(i,3)*x(11)+x(12))-data(i,5))^2);%[xp,yp]与[x,y,z]的转换误差
%     f(2*i-1,1)=(data(i,1)*x(1)+data(i,2)*x(2)+data(i,3)*x(3)+x(4))/(data(i,1)*x(9)+data(i,2)*x(10)+data(i,3)*x(11)+x(12))-data(i,4);%[xp]与[x,y,z]的转换误差（double float）
%     f(2*i,1)=(data(i,1)*x(5)+data(i,2)*x(6)+data(i,3)*x(7)+x(8))/(data(i,1)*x(9)+data(i,2)*x(10)+data(i,3)*x(11)+x(12))-data(i,5);%[yp]与[x,y,z]的转换误差（double float）
    Z=H*[data(i,1);data(i,2);data(i,3);1];
    f=[f;data(i,4)-Z(1)/Z(3);data(i,5)-Z(2)/Z(3)];
end
%旋转矩阵限制函数构造
% Hx=[x(1),x(2),x(3),x(4);x(5),x(6),x(7),x(8);x(9),x(10),x(11),x(12)];%坐标转移矩阵（double float）
% Bt=inv(A)*Hx;%计算外参矩阵（double float）
% Bt=Bt(:,1:3);%获取外参旋转矩阵（double float）
% Mx=Bt'*Bt-[1,0,0;0,1,0;0,0,1];%旋转矩阵限制1（double float）
% f(2*N+1,1)=100*(det(Bt)-1);%旋转矩阵限制2，添加1行优化函数（double float）
% k=2;%通过旋转矩阵限制1添加优化函数（uint8）
% for i=1:3%旋转矩阵限制1分解，添加9行优化函数
%     for j=1:3
%         f(2*N+k,1)=100*Mx(i,j);%（double float）
%         k=k+1;%循环计数
%     end
% end%最终添加10行优化函数。
%最终函数f有2*N+10行。
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%L-M算法部分，求解非线性优化问题，优化变量x。
Jf=jacobian(f);%计算函数雅可比矩阵（double float）
%%%%%%%%%%
% 检验用代码
% rank(Jf)
% first guess
% xk=[1;0;0;100;0;1;0;100;0;0;1];
% xk=[H(1,1);H(1,2);H(1,3);H(1,4);H(2,1);H(2,2);H(2,3);H(2,4);H(3,1);H(3,2);H(3,3);H(3,4)];
%%%%%%%%%%
%初始化
xk_ini=xk;%优化变量赋初值（double float）
% xk=xk.*(ones(12,1)+1*randn(12,1));%检验用代码
fk=double(subs(f,x,xk));%f函数赋值（double float）
Jfk=double(subs(Jf,x,xk));%Jf函数赋值（double float）
Dk=sqrt(diag(diag(Jfk'*Jfk)));%Dk赋初值（double float）
sigma=0.1;%sigma参数设置（float）
pk=1;%Pk赋初值（double float）
deltak=1;%步长赋初值（float）
i=0;%循环计数（uint16）
%loop
% while norm(pk)>0.1 %另一优化阈值 
while i<100 %以循环数作为优化阈值，优化阈值设置。
    i=i+1%循环计数 
    %step a,计算lambda：牛顿法权重参数
    Jfkpinv=pinv(Jfk);%求伪逆（double float）
    if norm(Dk*pinv(Jfk)*fk)<=(1+sigma)*deltak%判断下降梯度
        %梯度平稳，线性近似
        lambdak=0;%（double float）
        pk=-pinv(Jfk)*fk;%变化量pk（double float）
    else
        %梯度过陡，牛顿法拟合
        alpha=0;%lambda优化初值（double float）
        u=norm((Jfk*inv(Dk))'*fk)/deltak;%上确界计算（double float）
        palpha=psolution(alpha,fk,Jfk,Dk);%p_alpha计算(double float）
        qalpha=Dk*palpha;%q_alpha计算(double float）
        phi=norm(qalpha)-deltak;%phi计算(double float）
        dphi = dphisolution(alpha,fk,Jfk,Dk);%dphi/dlambda计算(double float）
        l=-phi/dphi;%下确界计算(double float）
        j=0;%循环初始化（uint16）
        while (abs(phi)>sigma*deltak)&&(j<100)%lambda优化循环
            j=j+1;%循环计数
            if alpha<=l||alpha>=u%判断是否超过取值范围
                alpha=(0.001*u>sqrt(l*u))*0.001*u+(0.001*u<=sqrt(l*u))*sqrt(l*u);%超过时对优化变量alpha进行调整
            end
            if phi<0%判断是否下降过少
                u=alpha;%上确界更新
            end
            l=l*(l>(alpha-phi/dphi))+(alpha-phi/dphi)*(l<=(alpha-phi/dphi));%下确界更新
            alpha=alpha-(phi+deltak)/deltak*phi/dphi;%alpha更新
            palpha=psolution(alpha,fk,Jfk,Dk);%p_alpha更新
            qalpha=Dk*palpha;%q_alpha更新
            phi=norm(qalpha)-deltak;%phi更新
            dphi=dphisolution(alpha,fk,Jfk,Dk);%dphi/dlambda更新
        end
        lambdak=alpha;%优化完成，lambda赋值
        pk = psolution(lambdak,fk,Jfk,Dk);%pk计算
    end
    %step b，步长评估，线性度计算
    fkp=double(subs(f,x,xk+pk));%变化后优化函数取值(double float）
    fkkp(:,i)=fkp'*fkp;
    rhok=((fk'*fk)-fkp'*fkp)/(fk'*fk-(fk+Jfk*pk)'*(fk+Jfk*pk)) ;%线性度计算(double float）
    %step c，优化变量更新
    if rhok>0.0001%线性度合适
        xk=xk+pk;%优化变量更新
        fk=double(subs(f,x,xk));%fk更新
        Jfk=double(subs(Jf,x,xk)); %Jfk更新
%%%%%%%%%%%%%%%
%检验用代码
%         rank(Jfk)
%%%%%%%%%%%%%%%
    end
    %step d，步长更新
    if rhok<=0.25%线性度过小，说明步长过大
        deltak=0.5*deltak;%步长缩小调整
    elseif (rhok>0.25&&rhok<0.75&&lambdak==0)||rhok>=0.75%线性度过大，说明步长过小
        deltak=2*norm(Dk*pk);%步长扩大调整
    end
    %step e，更新步长梯度
    Dk=(Dk>sqrt(diag(diag(Jfk'*Jfk)))).*Dk+(Dk<=sqrt(diag(diag(Jfk'*Jfk)))).*sqrt(diag(diag(Jfk'*Jfk)));%更新Dk
%%%%%%%%%%%%%%%
%检验用代码
%     norm(pk)
%%%%%%%%%%%%%%%
    xkk(:,i)=xk;%优化变量过程量存储，12×I矩阵，I为优化循环数（目前为1000）。
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%作图部分
%%%%%%%%%%%%%%%
%检验用代码
% xk
% fk0=sqrt(double(subs(f,x,xk_ini))'*double(subs(f,x,xk_ini)))
% fk=sqrt(double(subs(f,x,xk))'*double(subs(f,x,xk)))
%%%%%%%%%%%%%%%
xkk_ini=xk_ini*ones(1,size(xkk,2));%迭代精度计算中所用矩阵换算（12×I矩阵，double float）
y=diag(sqrt((xkk-xkk_ini)'*(xkk-xkk_ini)));%迭代精度计算，均方误差（double float）
plot(y)%成图
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%参数存储
Hx=double(subs(H,x,xk));%坐标转移矩阵（4×3矩阵，double float）
Bx=A\Hx;
save Hx Hx;%坐标转移矩阵存储。（输出）
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

