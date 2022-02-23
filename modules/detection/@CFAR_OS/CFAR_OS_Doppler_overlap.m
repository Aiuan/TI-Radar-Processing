function [N_obj, Ind_obj, noise_obj_an] = CFAR_OS_Doppler_overlap(obj, Ind_obj_Rag, sigCpml, sig_integ)
    % ������ΪOS CFAR������άCFAR
    maxEnable = obj.maxEnable; %�Ƿ����ֵ���״̬
    cellNum0 = obj.refWinSize; %��ȡ����Ĳο�������
    gapNum0 = obj.guardWinSize; %��ȡ����ı�����Ԫ����
    cellNum = cellNum0(2); %��ȡ������ά�Ĳο�������
    gapNum = gapNum0(2); %��ȡ������ά�ı���������
    K0 = obj.K0(2); %������ά������ϵ��

    rangeNumBins = size(sig_integ,1); %�����sigΪ2D���ݣ��ֱ��ʾÿ��chrip��������������ά�����Loop���������и��

    detected_Rag_Cell = unique(Ind_obj_Rag(:,1)); %��ȡ���ظ���Ŀ���������
    sig = sig_integ(detected_Rag_Cell,:); %ͳ������Ŀ���ѭ�������radarcube��������

    M_samp = size(sig, 1); %��ȡ����������������ά��⵽Ŀ��ĵ���
    N_pul = size(sig, 2); %��ȡLoop����

    gaptot = gapNum + cellNum; %�ο���

    N_obj = 0; %���Ŀ������ʼ��
    Ind_obj = []; %Ŀ��������ʼ��
    noise_obj_an = []; 
    vec = zeros(1,N_pul+gaptot*2);
    for k = 1:M_samp %��ÿ��Ŀ���
        detected_Rag_Cell_i = detected_Rag_Cell(k); %��ȡ��k��Ŀ���ľ���
        ind1 = find(Ind_obj_Rag(:,1) == detected_Rag_Cell_i);%��һ��ѭ����Ŀ��λ����Ŀ���ľ������ƥ��
        indR = Ind_obj_Rag(ind1, 2); %��������

        sigv=(sig(k,:)); %��ȡÿ��Ŀ������Ӧ�Ķ�����ά��Ϣ
        vec(1:gaptot) = sigv(end-gaptot+1:end); %���ұ߽���Ϣ��������߽�
        vec(gaptot+1: N_pul+gaptot) = sigv; %�м���Ϣ��sigv����
        vec(N_pul+gaptot+1:end) = sigv(1:gaptot); %����߽���Ϣ�������ұ߽�

        %CFAR����
        ind_loc_all = []; %����ά��������
        ind_loc_Dop = []; %������ά��������
        ind_obj_0 = 0;
        noiseEst = zeros(1,N_pul); %�������ƣ�����Ϊ������
        for j = 1+gaptot:N_pul+gaptot %ȥ�м�ԭʼ�����ݽ��д���
            cellInd = [j-gaptot:j-gapNum-1 j+gapNum+1:j+gaptot]; %��ȡ�ο���
            noiseEst(j-gaptot) = sum(vec(cellInd)); %�Բο������������
        end
        for j = 1+gaptot:N_pul+gaptot
            j0 = j - gaptot;
            cellInd = [j-gaptot:j-gapNum-1 j+gapNum+1:j+gaptot];
            cellInda = [j-gaptot: j-gapNum-1]; %ȡ���ο���
            cellIndb =[j+gapNum+1:j+gaptot]; %ȡ�Ҳ�ο���

    %         cellave1a = sum(vec(cellInda))/(cellNum); %���ο���������ά����ƽ��
    %         cellave1b = sum(vec(cellIndb))/(cellNum); %�Ҳ�ο���������ά����ƽ��
    %         cellave1 = min(cellave1a,cellave1b); %����ο���ƽ��ȡ��С        
            cell_k = sort(vec([cellInda,cellIndb]));
            cellave1 = cell_k(ceil(3/4*cellNum*2));
            maxInCell = max(vec(cellInd)); %��ȡ��ֵ����
            if maxEnable == 1 %���õ�Ԫ�Ƿ�Ϊ�ο����ڵķ�ֵ
                condition = ((vec(j)>K0*cellave1)) && ((vec(j)>maxInCell));
            else
                condition = vec(j)>K0*cellave1;
            end

            if condition == 1 %���j��Ԫ�ǲο����ķ�ֵ
                %���ü���Ƿ�������ռ���ص�
                if(find(indR == j0))
                    %����ص������������
                    ind_win = detected_Rag_Cell_i; %����j0Ŀ��Ķ�����ά��Ϣ
                    ind_loc_all = [ind_loc_all ind_win]; 
                    ind_loc_Dop = [ind_loc_Dop j0]; %����j0Ŀ�����������Ӧ�ھ���ά����
                end
            end
        end
        ind_obj_0 = [];

        if (length(ind_loc_all)>0)
            ind_obj_0(:,1) = ((ind_loc_all)); %indobj��һ�д������
            ind_obj_0(:,2) = ind_loc_Dop; %indobj�ڶ��д�ž�������
            if size(Ind_obj,1) == 0 %���û��������ֵ��IndObj
                Ind_obj = ind_obj_0;
            else%����
                %���¹�����Ϊ�˱����ظ��ļ���
                ind_obj_0_sum = ind_loc_all + 10000 * ind_loc_Dop;
                Ind_obj_sum = Ind_obj(:,1) + 10000 * Ind_obj(:,2);
                for ii= 1: length(ind_loc_all)
                    if (length(find(Ind_obj_sum == ind_obj_0_sum(ii)))==0)
                        Ind_obj = [Ind_obj ; ind_obj_0(ii,:)];
                    end
                end
            end
        end
    end

    N_obj = size(Ind_obj,1);

    %���òο�����
    cellNum = cellNum0(1); %����ά���ڳ���
    gapNum = gapNum0(1); %����ά������Ԫ
    gaptot = gapNum + cellNum;

    %��ȡÿ�����е�����ϵ��
    N_obj_valid = 0;
    Ind_obj_valid = [];
    for i_obj = 1:N_obj %��ÿ��Ŀ��    
        ind_range = Ind_obj(i_obj,1);
        ind_Dop = Ind_obj(i_obj,2);
        %�����źŹ���С��obj.powerThre�ļ���
        if (min(abs(sigCpml(ind_range, ind_Dop,:)).^2) < obj.powerThre)
            continue;
        end
        if ind_range <= gaptot%�ұ߽�
            cellInd = [ind_range+gapNum+1:ind_range+gaptot ind_range+gapNum+1:ind_range+gaptot];
        elseif ind_range >= rangeNumBins-gaptot+1%��߽�
            cellInd = [ind_range-gaptot:ind_range-gapNum-1 ind_range-gaptot:ind_range-gapNum-1];
        else %�м�߽�
            cellInd = [ind_range-gaptot: ind_range-gapNum-1 ind_range+gapNum+1:ind_range+gaptot];
        end

        N_obj_valid = N_obj_valid +1;
        noise_obj_an(:, i_obj) = reshape((mean(abs(sigCpml(cellInd, ind_Dop, :)).^2, 1)), obj.numAntenna, 1, 1);
        Ind_obj_valid(N_obj_valid,:) = Ind_obj(i_obj,:);    

    end

    N_obj = N_obj_valid;
    Ind_obj = Ind_obj_valid;
end






