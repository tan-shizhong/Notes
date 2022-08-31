function T = dh2matrix(a, alpha, d, theta)

%  T = dh2matrix(a, alpha, d, theta)
%  将标准DH参数转化为旋转矩阵

ct = cos(theta); 
st = sin(theta);
ca = cos(alpha); 
sa = sin(alpha);

T = [ct,   -st,   0,    a;
    st*ca, ct*ca, -sa,  -sa*d;
    st*sa, ct*sa, ca,   ca*d;
    0,  0,  0,  1];

end