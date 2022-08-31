function oula_xyz = matrix2xyz(T)
% oula_xyz = matrix2xyz(T)
% 将旋转矩阵转化为xyz欧拉角

oula_xyz(1) = T(1,4);
oula_xyz(2) = T(2,4);
oula_xyz(3) = T(3,4);

R_rot = T(1:3,1:3);

if R_rot(1,3) ~= 1 && R_rot(1,3) ~= -1    
    beta = asin(R_rot(1,3));
    gamma = -atan2(R_rot(1,2)/cos(beta),R_rot(1,1)/cos(beta));
    alpha = -atan2(R_rot(2,3)/cos(beta),R_rot(3,3)/cos(beta));
else
    gamma = 0;
    disp('---------------------奇异，取gamma=0---------------------');
    if R_rot(1,3) == 1
        beta = pi/2;
        alpha = gamma + atan2(R_rot(2,1),R_rot(3,1));
    else
        beta = -pi/2;
        alpha = -gamma + atan2(R_rot(2,1),R_rot(3,1));
    end
end

oula_xyz(4) = alpha;
oula_xyz(5) = beta;
oula_xyz(6) = gamma;

end

