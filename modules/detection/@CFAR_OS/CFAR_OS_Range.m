function [N_obj, Ind_obj, noise_obj, CFAR_SNR] = CFAR_OS_Range(obj, sig)
%% 本函数为OS CFAR距离维CFAR
%输入的sig为2D数据，分别表示每个chrip采样点数，Loop次数
cellNum = obj.refWinSize; %读取对象的参考窗
gapNum = obj.guardWinSize; %读取对象的保护单元
cellNum = cellNum(1); %距离维CFAR的参考窗长度
gapNum = gapNum(1); %距离维CFAR的保护单元
K0 = obj.K0(1);  %距离维的门限系数

M_samp = size(sig, 1); %获取采样点数
N_pul = size(sig, 2); %获取Loop次数

gaptot = gapNum + cellNum; %参考单元+保护单元（相对半参考窗而言）
N_obj = 0; %目标数目
Ind_obj = []; %目标在sig中的索引
noise_obj = []; %噪声存储
CFAR_SNR = []; %CFAR信噪比存储

discardCellLeft = obj.discardCellLeft; %丢弃左侧单元的数目
discardCellRight = obj.discardCellRight; %丢弃右侧单元的数目

for k = 1:N_pul %每次循环
    sigv = (sig(:,k))'; %提取每次循环的采样点数并转换为行向量
    vec = sigv(discardCellLeft+1:M_samp-discardCellRight); %提取去除近场和远场后的数据
    vecLeft = vec(1:(gaptot)); %提取左侧近场背景水平填补左边界
    vecRight = vec(end-(gaptot)+1:end);  %提取右侧远场背景水平填补右边界
    vec = [vecLeft vec vecRight]; %构建闭环检测单元
    for j = 1:(M_samp-discardCellLeft-discardCellRight)
        cellInd = [j-gaptot:j-gapNum-1 j+gapNum+1:j+gaptot]; %获取j检测单元的参考窗
        cellInd = cellInd + gaptot; %加上填补的边界长度
                                 
        cellInda = [j-gaptot:j-gapNum-1]; %左侧半参考窗
        cellInda = cellInda + gaptot; %加上填补的左边界长度
        cellIndb = [j+gapNum+1:j+gaptot]; %右侧参考窗
        cellIndb = cellIndb + gaptot; %加上填补的右边界长度
%         cellave1a =sum(vec(cellInda))/(cellNum); %对左侧参考窗采样值取平均
%         cellave1b =sum(vec(cellIndb))/(cellNum); %对右侧参考窗蚕养殖取平均
%         cellave1 = min(cellave1a,cellave1b); %选择平均值最小的一侧参考窗值
        cell_k = sort(vec([cellInda,cellIndb])); %进行排序
        cellave1 = cell_k(ceil(3/4*cellNum*2)); %选择第3/4参考窗的单元作为估计

        %if((j > discardCellLeft) && (j < (M_samp-discardCellRight)))
        if obj.maxEnable == 1 %进行峰值判断
            maxInCell = max(vec([cellInd(1):cellInd(end)])); %检查是否为局部最大值并输出
            if (vec(j+gaptot)>K0*cellave1 && ( vec(j+gaptot) >= maxInCell)) 
                %j+gaptot是指原始vec有效单元中的目标，判断检测单元是否大于门限水平
                N_obj = N_obj+1; %检测单元大于门限水平则目标数目+1
                Ind_obj(N_obj,:) = [j+discardCellLeft, k]; %记录下目标的位置（这里要加上删除的近场数据）
                noise_obj(N_obj) = cellave1; %保存杂波功率水平
                CFAR_SNR(N_obj) = vec(j+gaptot)/cellave1; %记录目标信噪比
            end
        else %不进行峰值判断，其余步骤与上相同
            if vec(j+gaptot)>K0*cellave1
                N_obj=N_obj+1;
                Ind_obj(N_obj,:)=[j+discardCellLeft, k];
                noise_obj(N_obj) = cellave1; %保存噪声水平
                CFAR_SNR(N_obj) = vec(j+gaptot)/cellave1;
            end
        end        
    end
end


%获取每个阵列的噪声方差系数
for i_obj = 1:N_obj %每个目标
    ind_range = Ind_obj(i_obj,1); %目标在sig的索引
    ind_Dop = Ind_obj(i_obj,2); %第几个循环
    if ind_range <= gaptot %左边界参考窗处理
        cellInd = [ind_range+gapNum+1:ind_range+gaptot ind_range+gapNum+1:ind_range+gaptot];
    elseif ind_range >= M_samp-gaptot+1 %右边界参考窗处理
        cellInd = [ind_range-gaptot:ind_range-gapNum-1 ind_range-gaptot:ind_range-gapNum-1];
    else%中间边界参考窗处理
        cellInd = [ind_range-gaptot:ind_range-gapNum-1 ind_range+gapNum+1:ind_range+gaptot];   
    end
end

