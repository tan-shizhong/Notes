function T_roty = roty(beta)

% T_roty = roty(beta)
% 绕Y轴旋转beta(弧度)时旋转矩阵

T_roty = [
    cos(beta),0,sin(beta);
    0,1,0;
    -sin(beta),0,cos(beta);
    ];

end