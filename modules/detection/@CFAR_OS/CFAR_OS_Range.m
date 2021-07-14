function [N_obj, Ind_obj, noise_obj, CFAR_SNR] = CFAR_OS_Range(obj, sig)
%% ������ΪOS CFAR����άCFAR
%�����sigΪ2D���ݣ��ֱ��ʾÿ��chrip����������Loop����
cellNum = obj.refWinSize; %��ȡ����Ĳο���
gapNum = obj.guardWinSize; %��ȡ����ı�����Ԫ
cellNum = cellNum(1); %����άCFAR�Ĳο�������
gapNum = gapNum(1); %����άCFAR�ı�����Ԫ
K0 = obj.K0(1);  %����ά������ϵ��

M_samp = size(sig, 1); %��ȡ��������
N_pul = size(sig, 2); %��ȡLoop����

gaptot = gapNum + cellNum; %�ο���Ԫ+������Ԫ����԰�ο������ԣ�
N_obj = 0; %Ŀ����Ŀ
Ind_obj = []; %Ŀ����sig�е�����
noise_obj = []; %�����洢
CFAR_SNR = []; %CFAR����ȴ洢

discardCellLeft = obj.discardCellLeft; %������൥Ԫ����Ŀ
discardCellRight = obj.discardCellRight; %�����Ҳ൥Ԫ����Ŀ

for k = 1:N_pul %ÿ��ѭ��
    sigv = (sig(:,k))'; %��ȡÿ��ѭ���Ĳ���������ת��Ϊ������
    vec = sigv(discardCellLeft+1:M_samp-discardCellRight); %��ȡȥ��������Զ���������
    vecLeft = vec(1:(gaptot)); %��ȡ����������ˮƽ���߽�
    vecRight = vec(end-(gaptot)+1:end);  %��ȡ�Ҳ�Զ������ˮƽ��ұ߽�
    vec = [vecLeft vec vecRight]; %�����ջ���ⵥԪ
    for j = 1:(M_samp-discardCellLeft-discardCellRight)
        cellInd = [j-gaptot:j-gapNum-1 j+gapNum+1:j+gaptot]; %��ȡj��ⵥԪ�Ĳο���
        cellInd = cellInd + gaptot; %������ı߽糤��
                                 
        cellInda = [j-gaptot:j-gapNum-1]; %����ο���
        cellInda = cellInda + gaptot; %���������߽糤��
        cellIndb = [j+gapNum+1:j+gaptot]; %�Ҳ�ο���
        cellIndb = cellIndb + gaptot; %��������ұ߽糤��
%         cellave1a =sum(vec(cellInda))/(cellNum); %�����ο�������ֵȡƽ��
%         cellave1b =sum(vec(cellIndb))/(cellNum); %���Ҳ�ο�������ֳȡƽ��
%         cellave1 = min(cellave1a,cellave1b); %ѡ��ƽ��ֵ��С��һ��ο���ֵ
        cell_k = sort(vec([cellInda,cellIndb])); %��������
        cellave1 = cell_k(ceil(3/4*cellNum*2)); %ѡ���3/4�ο����ĵ�Ԫ��Ϊ����

        %if((j > discardCellLeft) && (j < (M_samp-discardCellRight)))
        if obj.maxEnable == 1 %���з�ֵ�ж�
            maxInCell = max(vec([cellInd(1):cellInd(end)])); %����Ƿ�Ϊ�ֲ����ֵ�����
            if (vec(j+gaptot)>K0*cellave1 && ( vec(j+gaptot) >= maxInCell)) 
                %j+gaptot��ָԭʼvec��Ч��Ԫ�е�Ŀ�꣬�жϼ�ⵥԪ�Ƿ��������ˮƽ
                N_obj = N_obj+1; %��ⵥԪ��������ˮƽ��Ŀ����Ŀ+1
                Ind_obj(N_obj,:) = [j+discardCellLeft, k]; %��¼��Ŀ���λ�ã�����Ҫ����ɾ���Ľ������ݣ�
                noise_obj(N_obj) = cellave1; %�����Ӳ�����ˮƽ
                CFAR_SNR(N_obj) = vec(j+gaptot)/cellave1; %��¼Ŀ�������
            end
        else %�����з�ֵ�жϣ����ಽ��������ͬ
            if vec(j+gaptot)>K0*cellave1
                N_obj=N_obj+1;
                Ind_obj(N_obj,:)=[j+discardCellLeft, k];
                noise_obj(N_obj) = cellave1; %��������ˮƽ
                CFAR_SNR(N_obj) = vec(j+gaptot)/cellave1;
            end
        end        
    end
end


%��ȡÿ�����е���������ϵ��
for i_obj = 1:N_obj %ÿ��Ŀ��
    ind_range = Ind_obj(i_obj,1); %Ŀ����sig������
    ind_Dop = Ind_obj(i_obj,2); %�ڼ���ѭ��
    if ind_range <= gaptot %��߽�ο�������
        cellInd = [ind_range+gapNum+1:ind_range+gaptot ind_range+gapNum+1:ind_range+gaptot];
    elseif ind_range >= M_samp-gaptot+1 %�ұ߽�ο�������
        cellInd = [ind_range-gaptot:ind_range-gapNum-1 ind_range-gaptot:ind_range-gapNum-1];
    else%�м�߽�ο�������
        cellInd = [ind_range-gaptot:ind_range-gapNum-1 ind_range+gapNum+1:ind_range+gaptot];   
    end
end

