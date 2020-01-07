function [TABLE, GEOM, AIR] = fcnINPUT(filename)
% This function reads the desired input file

temp = strcat('GEOM\',filename);
run(temp)

if ~exist('TABLE','var')
    TABLE.VEH.temp = nan;
end
if ~fcnCOMPCHECK(TABLE, 'VEH')
TABLE.VEH.temp = nan;
end
end

