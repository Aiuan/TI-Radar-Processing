clear all;
close all;
clc;

load('./output/毫米波测试实验数据_mode90test5min.mat');

n_frame = size(xyz_all,2);

figure(1)
for i = 800:n_frame
    tic;
    tmp = xyz_all{i};
    x = tmp(:,1);
    y = tmp(:,2);
    z = tmp(:,3);
    velocity = tmp(:,4);
    reflection = tmp(:,6);
    
    scatter3(x,y,z,10,velocity,'filled');
    c = colorbar;
    c.Label.String = 'Velocity(m/s)';
    colormap(jet);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view(0,90);
    axis([-40 40 0 70 -10 10 -1 1]);
    
    minute = floor(i/600);
    second = mod(i,600)/10;
    text(20,60,0,[num2str(minute),'分',num2str(second),'秒']);
    
    toc;
    pause(0.05);
end

%% 
clear all;
close all;
clc;

load('./output/毫米波测试实验数据_mode90test5min.mat');

n_frame = size(xyz_all,2);

figure(1)
for i = 1:n_frame
    tic;
    tmp = xyz_all{i};
    x = tmp(:,1);
    y = tmp(:,2);
    z = tmp(:,3);
    velocity = tmp(:,4);
    reflection = tmp(:,6);
    
    scatter3(-x,y,z,10,reflection,'filled');
    c = colorbar;
    c.Label.String = 'Reflection(m/s)';
    colormap(jet);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view(0,90);
    axis([-40 40 0 70 -10 10 5 50]);
    
    toc;
    pause(0.05);
end

