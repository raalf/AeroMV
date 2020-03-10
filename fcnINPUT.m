function [TABLE, GEOM, AIR] = fcnINPUT(filename) %#ok<STOUT>
% This function reads the desired input file. The input file should be in
%   the GEOM folder
%
% INPUTS:
%   filename        - A string with filename. 
%                       Ex. filename = 'AscTec_Pelican';

% Run the input file in the GEOM folder
temp = strcat('GEOM\',filename);
run(temp)

% Check if table exist, else set it to NaNs
if ~exist('TABLE','var')
    TABLE.VEH.temp = nan;
end
% Cehck if table has a VEH structure. If not add it.
if ~fcnCOMPCHECK(TABLE, 'VEH')
    TABLE.VEH.temp = nan;
end
end

