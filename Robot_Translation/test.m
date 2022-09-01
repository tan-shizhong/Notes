clc
clear

%%
syms x y z a b c

oula = [x y z a b c];

matrix = zyz2matrix(oula)

oula_1 = [100 200 300 2.4 -1.4 3.0];

matrix_1 = xyz2matrix(oula_1)

oula_zyz = matrix2xyz(matrix_1)

oula_zyz_matlab = tform2eul(matrix_1,'ZYZ')