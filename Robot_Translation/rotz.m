function T_rotz = rotz(gamma)

% T_rotz = rotz(gamma)
% 绕Z轴旋转gamma(弧度)时旋转矩阵

T_rotz = [
    cos(gamma),-sin(gamma),0;
    sin(gamma),cos(gamma),0;
    0,0,1;
    ];

end