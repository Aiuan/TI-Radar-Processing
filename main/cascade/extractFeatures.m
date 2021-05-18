function [torso_velocity, limb_velocity, u, s, v] = extractFeatures(tmp, velocity_bin)
    
    % ����������ʸ��
    torso_velocity = zeros(1,size(tmp,2));
    for t = 1:size(torso_velocity, 2)
        torso_velocity(t) = (velocity_bin * tmp(:,t)) / sum(tmp(:,t));
    end
    %��֫������ʸ��
    limb_velocity = zeros(1,size(tmp,2));
    for t = 1:size(limb_velocity, 2)
        num = 0;
        for v = 1:size(velocity_bin,2)
            num = num + (velocity_bin(v) * tmp(v,t) - torso_velocity(t) * tmp(v,t))^2;
        end
        den = sum(tmp(:,t));
        limb_velocity(t) = sqrt(num / den);
    end
    %SVD����
    dim_svd = 3;
    [U,S,V] = svd(tmp);
    %ȡǰ3����Ϊ����
    u = U(:,1:dim_svd);
    s = S(1:dim_svd,1:dim_svd);
    v = V(1:dim_svd,:);
end