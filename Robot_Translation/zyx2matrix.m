function matrix = zyx2matrix(oula_zyx)

% matrix = zyx2matrix(oula_zyx)
% 将zyx欧拉角转化为旋转矩阵

R = rotz(oula_zyx(4))*roty(oula_zyx(5))*rotx(oula_zyx(6));

matrix = [
    [R, [oula_zyx(1);oula_zyx(2);oula_zyx(3)]]
    0,0,0,1
];

end