function q = xyz2quat(oula_angle)
% q = xyz2quat(oula_angle)
% 将xyz欧拉角转化为四元数，可以同时输入多个欧拉角，为n*3格式

angles = oula_angle;

cang = cos( angles/2 );
sang = sin( angles/2 );

q = [ cang(:,1).*cang(:,2).*cang(:,3) - sang(:,1).*sang(:,2).*sang(:,3), ...
            cang(:,1).*sang(:,2).*sang(:,3) + sang(:,1).*cang(:,2).*cang(:,3), ...
            cang(:,1).*sang(:,2).*cang(:,3) - sang(:,1).*cang(:,2).*sang(:,3), ...
            cang(:,1).*cang(:,2).*sang(:,3) + sang(:,1).*sang(:,2).*cang(:,3)];

end