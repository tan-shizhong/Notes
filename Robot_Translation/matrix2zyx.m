function oula_zyx = matrix2zyx(T)
% oula_zyx = matrix2zyx(T)
% 将旋转矩阵转化为zyx欧拉角

oula_zyx(1) = T(1,4);
oula_zyx(2) = T(2,4);
oula_zyx(3) = T(3,4);

R_rot = T(1:3,1:3);

if R_rot(3,1) ~= 1 && R_rot(3,1) ~= -1    
    beta = -asin(R_rot(3,1));
    gamma = atan2(R_rot(3,2)/cos(beta),R_rot(3,3)/cos(beta));
    alpha = atan2(R_rot(2,1)/cos(beta),R_rot(1,1)/cos(beta));
else
    gamma = 0;
    disp('---------------------奇异，取gamma=0---------------------');
    if R_rot(3,1) == 1
        beta = pi/2;
        alpha = gamma + atan2(R_rot(1,2),R_rot(1,3));
    else
        beta = -pi/2;
        alpha = -gamma + atan2(-R_rot(1,2),-R_rot(1,3));
    end
end

oula_zyx(4) = alpha;
oula_zyx(5) = beta;
oula_zyx(6) = gamma;

end

