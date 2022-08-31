function oula_zyz = matrix2zyz(T)
% oula_zyz = matrix2zyz(T)
% 将旋转矩阵转化为zyz欧拉角
oula_zyz(1) = T(1,4);
oula_zyz(2) = T(2,4);
oula_zyz(3) = T(3,4);

R_rot = T(1:3,1:3);

if R_rot(1,3) ~= 0 && R_rot(2,3) ~= 0 
    beta=atan2(sqrt(R_rot(3,1)*R_rot(3,1)+R_rot(3,2)*R_rot(3,2)),R_rot(3,3));
    gamma = atan2(R_rot(3,2)/sin(beta),-R_rot(3,1)/sin(beta));
    alpha = atan2(R_rot(2,3)/sin(beta),R_rot(1,3)/sin(beta));
else
    disp('---------------------奇异，beta=0---------------------');
    beta=0;
    alpha=0;
    gamma=atan2(-R_rot(1,2),R_rot(1,1));
end
oula_zyz(4)=alpha;
oula_zyz(5)=beta;
oula_zyz(6)=gamma;

end

