function matrix = zyz2matrix(oula_zyz)

% tfmatrix = zyz2matrix(oula_zyz)
% 将zyz欧拉角转化为旋转矩阵

R = rotz(oula_zyz(4))*roty(oula_zyz(5))*rotz(oula_zyz(6));

matrix = [
    [R, [oula_zyz(1);oula_zyz(2);oula_zyz(3)]]
    0,0,0,1
];

end