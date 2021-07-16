%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%���ܣ�ʵ�����ϱ궨
%���룺
% data:�궨���ݾ��� N��5����NΪ��������������20��������ÿ��������ǰ��������ʾ�״�����[x,y,z]��float��������������ʾ��Ӧ��ͼ�������[xp,yp]��uint16��;
% A:�ڲξ��� 3��3����double float��
% B:��ξ��� 3��4����float��
%�����
% H:����ת�ƾ��� 3��4����double float��
%��غ�����
%psolution(alpha,fk,Jfk,Dk)��psolution(lambda,f,Jf,D)��p����С���˽�
%dphi = dphisolution(lambda,f,Jf,D)����dphi/dlambda
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%��ʼ��
clear all;
close all;
x=[sym('thetax');sym('thetay');sym('thetaz');sym('tx');sym('ty');sym('tz')]; %���庯������ x����12��������x(1)~x(12)��double float��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ������������
% N=input('������������N��ֵ:\n');
% N=25;
% for i=1:N
%     fprintf('�������%d����������[xr,yr,zr,xp,yp]:',i)
%     data(i,:)=input('\n');
% end
%ת�ƾ����趨
% A=[800,0,200;0,800,200;0,0,1];     %�ڲξ���
% R=[1,0,0;0,1,0;0,0,1];             %��ת����
% t=[1;1;1];                         %ƽ�ƾ���
% H=A*[R,t];                         %��Ӧ����
%��������
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
%ʵ�����ݶ�ȡ����������������ķ���������������ȡ1��
% load('data_calibration.mat');%��ȡ���ڱ궨������
% data=data_calibration;%�궨���ݾ���ֵ�����룬double float��
% N=size(data,1);%��ȡ���ݳ��ȣ�uint8��
% load('A.mat');
% A=[680.52,0,279.52;0,683.47,266.69;0,0,1];%�ڲξ���ֵ�����룬double float��
Rx=[1,0,0;0,cos(x(1)),sin(x(1));0,-sin(x(1)),cos(x(1))];
Ry=[cos(x(2)),0,-sin(x(2));0,1,0;sin(x(2)),0,cos(x(2))];
Rz=[cos(x(3)),-sin(x(3)),0;sin(x(3)),cos(x(3)),0;0,0,1];
B=[Rx*Ry*Rz,[x(4);x(5);x(6)]];
% load('B_initial.mat');%��ȡ��ξ���
% B=B_initial;%��ξ����ƣ����룬double float��
% H_initial=A*B;%�����ʼ����ת�ƾ���double float��
% save H_initial H_initial;%�����ʼ����ת�ƾ���
% H=reshape(H_initial',12,1);%�����ع�����Ϊ12��1��double float��

% xk=H;%������ֵ��ֵ��double float��
% A = [964.0495477017739,0,0;0,964.7504668505122,0;613.3463725824778,377.1040664564536,1]; % intrinsic of our logi camera 1280*720
A = [1445.63012662922 0	0; 0 1445.10705595391 0; 920.902144019657 564.645986732188 1];% intrinsic of our logi camera 1920*1080

xk=[-pi/2;0;0;0;-0.07;-0.04];
A=A';

data_xyz=xlsread('20210626match_radar_results.xlsx');
load 20210626match_image_points.mat
data(:, 1:3)=data_xyz(:, 2:4);
save calibdata data


N=size(data,1);%��ȡ���ݳ��ȣ�uint8��

% �״���ǰΪy������Ϊx������Ϊz
% 

H=A*B;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%�Ż���������
f=[];
for i=1:N%�����Ż���������
%     f(i,1)=sqrt(((data(i,1)*x(1)+data(i,2)*x(2)+data(i,3)*x(3)+x(4))/(data(i,1)*x(9)+data(i,2)*x(10)+data(i,3)*x(11)+x(12))-data(i,4))^2+((data(i,1)*x(5)+data(i,2)*x(6)+data(i,3)*x(7)+x(8))/(data(i,1)*x(9)+data(i,2)*x(10)+data(i,3)*x(11)+x(12))-data(i,5))^2);%[xp,yp]��[x,y,z]��ת�����
%     f(2*i-1,1)=(data(i,1)*x(1)+data(i,2)*x(2)+data(i,3)*x(3)+x(4))/(data(i,1)*x(9)+data(i,2)*x(10)+data(i,3)*x(11)+x(12))-data(i,4);%[xp]��[x,y,z]��ת����double float��
%     f(2*i,1)=(data(i,1)*x(5)+data(i,2)*x(6)+data(i,3)*x(7)+x(8))/(data(i,1)*x(9)+data(i,2)*x(10)+data(i,3)*x(11)+x(12))-data(i,5);%[yp]��[x,y,z]��ת����double float��
    Z=H*[data(i,1);data(i,2);data(i,3);1];
    f=[f;data(i,4)-Z(1)/Z(3);data(i,5)-Z(2)/Z(3)];
end
%��ת�������ƺ�������
% Hx=[x(1),x(2),x(3),x(4);x(5),x(6),x(7),x(8);x(9),x(10),x(11),x(12)];%����ת�ƾ���double float��
% Bt=inv(A)*Hx;%������ξ���double float��
% Bt=Bt(:,1:3);%��ȡ�����ת����double float��
% Mx=Bt'*Bt-[1,0,0;0,1,0;0,0,1];%��ת��������1��double float��
% f(2*N+1,1)=100*(det(Bt)-1);%��ת��������2�����1���Ż�������double float��
% k=2;%ͨ����ת��������1����Ż�������uint8��
% for i=1:3%��ת��������1�ֽ⣬���9���Ż�����
%     for j=1:3
%         f(2*N+k,1)=100*Mx(i,j);%��double float��
%         k=k+1;%ѭ������
%     end
% end%�������10���Ż�������
%���պ���f��2*N+10�С�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%L-M�㷨���֣����������Ż����⣬�Ż�����x��
Jf=jacobian(f);%���㺯���ſɱȾ���double float��
%%%%%%%%%%
% �����ô���
% rank(Jf)
% first guess
% xk=[1;0;0;100;0;1;0;100;0;0;1];
% xk=[H(1,1);H(1,2);H(1,3);H(1,4);H(2,1);H(2,2);H(2,3);H(2,4);H(3,1);H(3,2);H(3,3);H(3,4)];
%%%%%%%%%%
%��ʼ��
xk_ini=xk;%�Ż���������ֵ��double float��
% xk=xk.*(ones(12,1)+1*randn(12,1));%�����ô���
fk=double(subs(f,x,xk));%f������ֵ��double float��
Jfk=double(subs(Jf,x,xk));%Jf������ֵ��double float��
Dk=sqrt(diag(diag(Jfk'*Jfk)));%Dk����ֵ��double float��
sigma=0.1;%sigma�������ã�float��
pk=1;%Pk����ֵ��double float��
deltak=1;%��������ֵ��float��
i=0;%ѭ��������uint16��
%loop
% while norm(pk)>0.1 %��һ�Ż���ֵ 
while i<100 %��ѭ������Ϊ�Ż���ֵ���Ż���ֵ���á�
    i=i+1%ѭ������ 
    %step a,����lambda��ţ�ٷ�Ȩ�ز���
    Jfkpinv=pinv(Jfk);%��α�棨double float��
    if norm(Dk*pinv(Jfk)*fk)<=(1+sigma)*deltak%�ж��½��ݶ�
        %�ݶ�ƽ�ȣ����Խ���
        lambdak=0;%��double float��
        pk=-pinv(Jfk)*fk;%�仯��pk��double float��
    else
        %�ݶȹ�����ţ�ٷ����
        alpha=0;%lambda�Ż���ֵ��double float��
        u=norm((Jfk*inv(Dk))'*fk)/deltak;%��ȷ����㣨double float��
        palpha=psolution(alpha,fk,Jfk,Dk);%p_alpha����(double float��
        qalpha=Dk*palpha;%q_alpha����(double float��
        phi=norm(qalpha)-deltak;%phi����(double float��
        dphi = dphisolution(alpha,fk,Jfk,Dk);%dphi/dlambda����(double float��
        l=-phi/dphi;%��ȷ�����(double float��
        j=0;%ѭ����ʼ����uint16��
        while (abs(phi)>sigma*deltak)&&(j<100)%lambda�Ż�ѭ��
            j=j+1;%ѭ������
            if alpha<=l||alpha>=u%�ж��Ƿ񳬹�ȡֵ��Χ
                alpha=(0.001*u>sqrt(l*u))*0.001*u+(0.001*u<=sqrt(l*u))*sqrt(l*u);%����ʱ���Ż�����alpha���е���
            end
            if phi<0%�ж��Ƿ��½�����
                u=alpha;%��ȷ�����
            end
            l=l*(l>(alpha-phi/dphi))+(alpha-phi/dphi)*(l<=(alpha-phi/dphi));%��ȷ�����
            alpha=alpha-(phi+deltak)/deltak*phi/dphi;%alpha����
            palpha=psolution(alpha,fk,Jfk,Dk);%p_alpha����
            qalpha=Dk*palpha;%q_alpha����
            phi=norm(qalpha)-deltak;%phi����
            dphi=dphisolution(alpha,fk,Jfk,Dk);%dphi/dlambda����
        end
        lambdak=alpha;%�Ż���ɣ�lambda��ֵ
        pk = psolution(lambdak,fk,Jfk,Dk);%pk����
    end
    %step b���������������Զȼ���
    fkp=double(subs(f,x,xk+pk));%�仯���Ż�����ȡֵ(double float��
    fkkp(:,i)=fkp'*fkp;
    rhok=((fk'*fk)-fkp'*fkp)/(fk'*fk-(fk+Jfk*pk)'*(fk+Jfk*pk)) ;%���Զȼ���(double float��
    %step c���Ż���������
    if rhok>0.0001%���ԶȺ���
        xk=xk+pk;%�Ż���������
        fk=double(subs(f,x,xk));%fk����
        Jfk=double(subs(Jf,x,xk)); %Jfk����
%%%%%%%%%%%%%%%
%�����ô���
%         rank(Jfk)
%%%%%%%%%%%%%%%
    end
    %step d����������
    if rhok<=0.25%���Զȹ�С��˵����������
        deltak=0.5*deltak;%������С����
    elseif (rhok>0.25&&rhok<0.75&&lambdak==0)||rhok>=0.75%���Զȹ���˵��������С
        deltak=2*norm(Dk*pk);%�����������
    end
    %step e�����²����ݶ�
    Dk=(Dk>sqrt(diag(diag(Jfk'*Jfk)))).*Dk+(Dk<=sqrt(diag(diag(Jfk'*Jfk)))).*sqrt(diag(diag(Jfk'*Jfk)));%����Dk
%%%%%%%%%%%%%%%
%�����ô���
%     norm(pk)
%%%%%%%%%%%%%%%
    xkk(:,i)=xk;%�Ż������������洢��12��I����IΪ�Ż�ѭ������ĿǰΪ1000����
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%��ͼ����
%%%%%%%%%%%%%%%
%�����ô���
% xk
% fk0=sqrt(double(subs(f,x,xk_ini))'*double(subs(f,x,xk_ini)))
% fk=sqrt(double(subs(f,x,xk))'*double(subs(f,x,xk)))
%%%%%%%%%%%%%%%
xkk_ini=xk_ini*ones(1,size(xkk,2));%�������ȼ��������þ����㣨12��I����double float��
y=diag(sqrt((xkk-xkk_ini)'*(xkk-xkk_ini)));%�������ȼ��㣬������double float��
plot(y)%��ͼ
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%�����洢
Hx=double(subs(H,x,xk));%����ת�ƾ���4��3����double float��
Bx=A\Hx;
save Hx Hx;%����ת�ƾ���洢���������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

