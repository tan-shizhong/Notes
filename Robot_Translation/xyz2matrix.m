function matrix = xyz2matrix(oula_xyz)

% matrix = xyz2matrix(oula_xyz)
% 将xyz欧拉角转化为旋转矩阵

R = rotx(oula_xyz(4))*roty(oula_xyz(5))*rotz(oula_xyz(6));

matrix = [
    [R, [oula_xyz(1);oula_xyz(2);oula_xyz(3)]]
    0,0,0,1
];

end