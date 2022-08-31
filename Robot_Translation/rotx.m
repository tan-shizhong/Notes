function T_rotx = rotx(alpha)

% T_rotx = rotx(alpha)
% 绕X轴旋转alpha(弧度)时旋转矩阵

T_rotx = [
    1,0,0;
    0,cos(alpha),-sin(alpha);
    0,sin(alpha),cos(alpha);
    ];

end