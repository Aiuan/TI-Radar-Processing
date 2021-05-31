function pixel_coordinate = projection(xyz, remove_distance_min,remove_distance_max)
    

    %移除近场、远场噪声
    xyz_use = xyz(xyz(:,5)>=remove_distance_min,:);
    xyz_use = xyz_use(xyz_use(:,5)<=remove_distance_max,:);    
    
    
%     %舟山标定矩阵
%     H = [978.427720340050, 589.993462843703, 13.1229972166119, -0.840684195154766;...
%             14.7696499738007, 388.888646655374, -959.947091338099, -137.871686789255;
%             0.0239198127750308, 0.999635770515324, 0.0124967541002258, 0.0523119353667915];
%         
        
%     20210513-20210514玉泉标定矩阵
    H = [296.017605935561, 835.703097555440, 33.2444346178279, -386.776196365743;...
        -196.067005960755, 455.292940171630, -575.779148193414, 184.623265383036;...
        -0.395085889573456, 0.917177372733613, 0.0518922615176608, 0.788409932235097];

    

    x = xyz_use(:,1);
    y = xyz_use(:,2);
    z = -xyz_use(:,3);
    
    radar_coordinate = [x y z ones(size(x))]';
    
    img_coordinate = H * radar_coordinate;
    
    pixel_coordinate = [img_coordinate(1,:)./img_coordinate(3,:);...%u
                        img_coordinate(2,:)./img_coordinate(3,:);%v
                        xyz_use(:,4)'];%velocity
    
    
end