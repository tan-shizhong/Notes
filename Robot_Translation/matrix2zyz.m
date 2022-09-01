function oula_zyz = matrix2zyz(T)
% oula_zyz = matrix2zyz(T)
% 将旋转矩阵转化为zyz欧拉角
oula_zyz(1) = T(1,4);
oula_zyz(2) = T(2,4);
oula_zyz(3) = T(3,4);

Zaxis=T(1:3,3);

if abs(Zaxis(1))<0.0001 && abs(Zaxis(2))<0.0001
    if Zaxis(3)>0.9
        %disp('---------------------奇异，beta=0---------------------');
        beta=0;
        alpha=0;
        gamma=atan2(-T(1,2),T(1,1));
    else 
        %disp('---------------------奇异，beta=pi---------------------');
        beta=pi;
        alpha=0;
        gamma=atan2(T(1,2),-T(1,1));
    end
else
    beta=atan2(sqrt(T(3,1)*T(3,1)+T(3,2)*T(3,2)),Zaxis(3));
    alpha=atan2(Zaxis(2),Zaxis(1));
    gamma=atan2(T(3,2),-T(3,1));
end

oula_zyz(4)=alpha;
oula_zyz(5)=beta;
oula_zyz(6)=gamma;

end

