% find corner reflector in pixel coordinate
clear all;  close all; clc;

Pic_path='K:\ourDataset\20210626match_radar_camera\image\';
image_list = getImageList(Pic_path);

for i=1:length(image_list) 
    image_path = image_list(i).path;
    image=imread(image_path);
    imshow(image)%显示该图
    title(image_path);
    set(gcf,'outerposition',get(0,'screensize'));%使该图显示最大化，便于取点
    [data(i,4),data(i,5)] = ginput
end

save 20210626match_image_points data