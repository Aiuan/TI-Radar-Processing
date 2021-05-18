% author aifuyuan
% this code is used to compute the vitural array of the (16RX n TX) array 
clear all;
close all;
clc;

% 4块AWR2243芯片 3T4R
% 芯片1 RXID： 1  2  3  4 TXID： 1  2  3
% 芯片2 RXID： 5  6  7  8 TXID： 4  5  6
% 芯片3 RXID： 9 10 11 12 TXID： 7  8  9
% 芯片4 RXID：13 14 15 16 TXID：10 11 12
% 芯片在板上的位置： 芯片4 芯片1 芯片3 芯片2
% 天线在板上的位置
TI_Cascade_TX_ID = [12 11 10 3 2 1 9 8 7 6 5 4 ]; %TX channel order on TI 4-chip cascade EVM
TI_Cascade_RX_ID = [13 14 15 16 1 2 3 4 9 10 11 12 5 6 7 8 ]; %RX channel order on TI 4-chip cascade EVM

%每根天线的坐标按正序排列
TI_Cascade_TX_position_azi = [11 10 9 32 28 24 20 16 12 8 4 0 ];%12 TX antenna azimuth position on TI 4-chip cascade EVM
TI_Cascade_TX_position_ele = [6 4 1 0 0 0 0 0 0 0 0 0];%12 TX antenna elevation position on TI 4-chip cascade EVM
TI_Cascade_RX_position_ele = zeros(1,16);%16 RX antenna elevation position on TI 4-chip cascade EVM
TI_Cascade_RX_position_azi = [ 11:14 50:53 46:49 0:3  ];

%选择使用的收发天线
selectTX = 1:4;
selectRX = 1:16;

num_TX = size(selectTX,2);
num_RX = size(selectRX,2);

%% relative_azimuth_cordinate RX相对于每根TX的方位角坐标
relative_azimuth_cordinate = zeros(num_TX, num_RX);
for i=1:num_TX
    relative_azimuth_cordinate(i,:) = TI_Cascade_RX_position_azi - TI_Cascade_TX_position_azi(i);
end

virtual_azimuth_cordinate = unique(relative_azimuth_cordinate);
%% relative_elevation_cordinate RX相对于每根TX的俯仰角坐标
relative_elevation_cordinate = zeros(num_TX, num_RX);
for i=1:num_TX
    relative_elevation_cordinate(i,:) = TI_Cascade_RX_position_ele - TI_Cascade_TX_position_ele(i);
end

virtual_elevation_cordinate = unique(relative_elevation_cordinate);
%% virtual array
% 以TX（0,0）—RX（0，0）作为基准
virtual_array = [];
count = 0;

for t=1:num_TX
    for r=1:num_RX
        transfer = [TI_Cascade_TX_position_azi(t) TI_Cascade_TX_position_ele(t)];
        recieve = [TI_Cascade_RX_position_azi(r) TI_Cascade_RX_position_ele(r)];
        temp = transfer - recieve;
        
        if t == 1 && r == 1
            virtual_array(count+1,:) = temp;
            count = count+1;
        else
            idx = find(virtual_array(:,1)==temp(1,1));
            idx2 = find(virtual_array(idx,2)==temp(1,2));
            if size(idx2,1) == 0
                virtual_array(count+1,:) = temp;
                count = count+1;
            end            
        end
        
        
    end
end


figure();
scatter(virtual_array(:,1) - min(virtual_array(:,1)) + 1, virtual_array(:,2) - min(virtual_array(:,2)) + 1);
grid on;
xlabel('azimuth');
ylabel('elevation')
title([num2str(num_TX),'TX ',num2str(num_RX), 'RX']);

disp('===========================================')
fprintf('Use %d TX, %d RX\n',num_TX,num_RX);
fprintf('Used TX_id: ');
for i = 1:num_TX
    if i ~= num_TX
        fprintf('%d, ', selectTX(i));
    else
        fprintf('%d\n', selectTX(i));
    end      
end
fprintf('Used RX_id: ');
for i = 1:num_RX
    if i ~= num_RX
        fprintf('%d, ', selectRX(i));
    else
        fprintf('%d\n', selectRX(i));
    end      
end
fprintf('Azimuth antenna size: %d\n', max(virtual_array(:,1)) - min(virtual_array(:,1)) + 1)
fprintf('Elevation antenna size: %d\n', max(virtual_array(:,2)) - min(virtual_array(:,2)) + 1)