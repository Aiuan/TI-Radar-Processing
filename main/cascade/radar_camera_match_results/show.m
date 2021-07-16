%% Îó²îÖ±·½Í¼
clear all; close all; clc;

load calibdata
load('Hx.mat', 'Hx');
H=Hx;

Pic_path='K:\ourDataset\20210626match_radar_camera\image\';
image_list = getImageList(Pic_path);

for i=1:length(image_list) 
    data1=data(i,1:3)';
    I=size(data1,2);
    data1=[data1;ones(1,I)];
    xc=H*data1*(diag(1./([0,0,1]*H*data1)));
    xc=floor(xc);
    xc(3,:)=[];
    error(i)=norm(xc - data(i,4:5)');
end
sum(error)
bar(error);
grid on;

%% ÖðÖ¡¼ì²é
clear all; close all; clc;

load calibdata
load('Hx.mat', 'Hx');
H=Hx;

Pic_path='K:\ourDataset\20210626match_radar_camera\image\';
image_list = getImageList(Pic_path);

figure();
for i=1:length(image_list) 
    data1=data(i,1:3)';
    I=size(data1,2);
    data1=[data1;ones(1,I)];
    xc=H*data1*(diag(1./([0,0,1]*H*data1)));
    xc=floor(xc);
    xc(3,:)=[];
    error(i)=norm(xc - data(i,4:5)');
    
    image = imread(image_list(i).path);
    imshow(image);
    hold on;
    scatter(data(i,4), data(i,5), 'filled', 'g');
    scatter(xc(1), xc(2), 'filled', 'r');    
    hold off;
    title([image_list(i).name, '    Error = ',num2str(error(i)), '    GT = (', num2str(data(i,4)), ',', num2str(data(i,4)), ')    Transfer = (', num2str(xc(1)), ',', num2str(xc(2)), ')']);
    
    %ç­‰å¾…æŒ‰é”®         
    key = waitforbuttonpress;
    while(key==0)
        key = waitforbuttonpress;
    end
end



