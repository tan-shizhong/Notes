function T_FK = fk(dhparams, q)

% T_FK = fk(dhparams, q)
% 六自由度机器人标准DH参数正解
% dhparams: a,alpha,d,theta

T0_1 = DH2tform(dhparams(1,1),dhparams(1,2),dhparams(1,3),dhparams(1,4) + q(1));
T1_2 = DH2tform(dhparams(2,1),dhparams(2,2),dhparams(2,3),dhparams(2,4) + q(2));
T2_3 = DH2tform(dhparams(3,1),dhparams(3,2),dhparams(3,3),dhparams(3,4) + q(3));
T3_4 = DH2tform(dhparams(4,1),dhparams(4,2),dhparams(4,3),dhparams(4,4) + q(4));
T4_5 = DH2tform(dhparams(5,1),dhparams(5,2),dhparams(5,3),dhparams(5,4) + q(5));
T5_6 = DH2tform(dhparams(6,1),dhparams(6,2),dhparams(6,3),dhparams(6,4) + q(6));

T_FK{1} = T0_1;
T_FK{2} = T_FK{1}*T1_2;
T_FK{3} = T_FK{2}*T2_3;
T_FK{4} = T_FK{3}*T3_4;
T_FK{5} = T_FK{4}*T4_5;
T_FK{6} = T_FK{5}*T5_6;

end