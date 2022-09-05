function xyzijk = ug2xyzijk(loadname)
% xyzijk = ug2xyzijk(loadname)
% 将UG导出的刀路文件转化为xyzijk坐标点

fid = fopen(loadname,'r');
i = 1;
while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line),break,end
    if length(line)>1 && strcmp(line(1:4),'GOTO')
        temp = str2num(line(6:end));
        if length(temp) == 3
            temp(4:6) = xyzijk(i-1,4:6);
        end
        xyzijk(i,:) = temp;
        i = i + 1;
    end
end