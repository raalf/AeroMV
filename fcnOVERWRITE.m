function [AIR,GEOM,TABLE,STATE] = fcnOVERWRITE(OVERWRITE,AIR,GEOM,TABLE,STATE)
%fcnOVERWRITE overwrites any variables from the input variables using the
%   values in the OVERWRITE structure. The inputs of the structures of
%   AIR, GEOM, TABLE and STATE can be updated.
%
% INPUTS:
%   OVERWRITE   - A structure with variables the must be changed from the
%           input values. An example would be: 
%                   OVERWRITE.GEOM.VEH.vecCG = [0 0 0];
%                   OVERWRITE.GEOM.VEH.ARM.valLENGTH = 0.3;
%                   OVERWRITE.GEOM.ROTOR.valNUMB = 3;
%                   OVERWRITE.AIR.density = 1.5;
%   AIR, GEOM, TABLE, STATE - These are the input variables that can be
%   overwriten by the variables in the OVERWRITE structure.
%
% OUTPUTS:
%   AIR, GEOM, TABLE, STATE structures with the updated OVERWRITE values


if fcnCOMPCHECK(OVERWRITE, 'GEOM')
    GEOM = fcnUPDATESTRUCTVAR(OVERWRITE.GEOM,GEOM);
end
if fcnCOMPCHECK(OVERWRITE, 'AIR')
    AIR = fcnUPDATESTRUCTVAR(OVERWRITE.AIR,AIR);
end
if fcnCOMPCHECK(OVERWRITE, 'TABLE')
    TABLE = fcnUPDATESTRUCTVAR(OVERWRITE.TABLE,TABLE);
end
if fcnCOMPCHECK(OVERWRITE, 'STATE')
    STATE = fcnUPDATESTRUCTVAR(OVERWRITE.STATE,STATE);
end

