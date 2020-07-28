function [vecCD,vecDRAG,vecCL,vecLIFT, TABLE] = fcnVEHFORCE(STATE, INFO, AIR, TABLE)
% This function calculates the forces acting on a vehicle component
% depending on the experienced velocity and the component shape
%
%
% OUTPUTS:
%   vecCD       - Component drag coefficient
%   vecDRAG     - Componet drag force value
%   vecCL       - Componet lift coefficients
%   vecLIFT     - Component lift force value
%   TABLE       - Updated table structure with data that wasnt read before
% 
% NOTE: these outputs are vectors with because some components have
% multiple of the same component. Ex. there may be 4 geometrically
% identical arm and thus there would be 4 entries in the output vectors.

%% Calculate drag for a sphere
% Required information in the INFO structure:
%   valDIAM - sphere diameter
%   strBODYNAME - body name for table lookup
if strcmpi(INFO.strTYPE,'Sphere')
    
    % Get Sphere Data Table
    if ~fcnCOMPCHECK(TABLE.VEH, 'matSPHERE')
        temp = load('TABLES\Sphere.mat');
        temp = struct2cell(temp);
        TABLE.VEH.matSPHERE = temp{1,1};
    end
    
    % Computre Reynolds number
    vecRE = STATE.VEL_MAG*INFO.valDIAM/AIR.kinvisc;
    
    % Interpolate CD using a linear interpolation in the logarithmic scale
    tempCD = interp1(log(TABLE.VEH.matSPHERE(:,1)),log(TABLE.VEH.matSPHERE(:,2)),log(vecRE));
    vecCD = exp(tempCD);
    vecCD(vecRE==0) = 0; % If Reynolds number is 0, drag is 0
    vecDRAG = vecCD.*(0.5.*AIR.density.*(STATE.VEL_MAG.^2).*(pi*INFO.valDIAM^2/4));
    vecDRAG(vecRE==0) = 0;
    vecCL = 0;
    vecLIFT = 0;
    
%% Calculate drag for an ellipsoid with in turbulent flow
% Required information in the INFO structure:
%   valHEIGHT - ellipsoid height
%   valLENGTH - ellipsoid length
elseif strcmpi(INFO.strTYPE,'Ellipsoid')
    
    if ~fcnCOMPCHECK(TABLE.VEH, 'Ellipsoid')
        tempAREA = pi*(INFO.valHEIGHT^2)/4;
        tempLentoDia = INFO.valLENGTH/INFO.valHEIGHT;
        
        %   NOTE: May need to use this during ascent/descent
        %     tempAREA = pi*(GEOM.VEH.BODY.valLENGTH^2)/4;
        %     tempLentoDia = GEOM.VEH.BODY.valHEIGHT/GEOM.VEH.BODY.valLENGTH;
        
        % Ellipsoid table
        TABLE.VEH.Ellipsoid.LENDIARATIO = [0.75 2 8];
        TABLE.VEH.Ellipsoid.DRAG = [0.5 0.2; 0.27 0.13; 0.2 0.08];
    end
    
    % Compute drag
    vecRE = STATE.VEL_MAG*INFO.valLENGTH/AIR.kinvisc;
    vecCD = interp1(TABLE.VEH.Ellipsoid.LENDIARATIO,TABLE.VEH.Ellipsoid.DRAG(:,1),tempLentoDia,'PCHIP');
    vecDRAG = vecCD*(0.5.*AIR.density.*(RUN.valVEL.^2).*tempAREA);
    vecDRAG(vecRE==0) = 0; % If Reynolds number is 0, drag is 0
    vecCL = 0;
    vecLIFT = 0;
    % Reference info
    % Anderson says CD at Re 1e5 is 0.12
    % Random masters thesis at Re 1e5 is around 0.13
    % Random textbook says below Re 2e5 is 0.5 and above Re 2e6 is 0.2

%% Calculate drag for an ellipsoid with in turbulent flow
% Required information in the INFO structure:
%   valDIAM - cylinder diameter
%   valLENGTH - cylinder length  
%   valAOA - Angle of cylinder (optional)
elseif strcmpi(INFO.strTYPE,'Cylinder')
    
    cyl_dir = INFO.matBEGIN-INFO.matEND;
    cyl_dir = cyl_dir./sqrt(cyl_dir(:,1).^2+cyl_dir(:,2).^2+cyl_dir(:,3).^2);
    valLENGTH = sqrt(cyl_dir(:,1).^2+cyl_dir(:,2).^2+cyl_dir(:,3).^2);
    numCOMP = size(INFO.matEND,1); % Number of components
    matVEL = repmat(STATE.VEL_B,numCOMP,1);
    valAOA = acos(dot(matVEL,cyl_dir,2)./(STATE.VEL_MAG)); % XXXX Must be revisited
    idx = valAOA > pi/2;
    valAOA(idx) = pi - valAOA(idx);

    if ~fcnCOMPCHECK(TABLE.VEH, 'matCYLINDER')
        temp = load(strcat('TABLES\Cylinder'));
        temp = struct2cell(temp);
        TABLE.VEH.matCYLINDER = temp{1,1};
    end
    
    vecRE = STATE.VEL_MAG*INFO.valDIAM/AIR.kinvisc;
    
    % Interpolate CD using a linear interpolation in the logarithmic scale
    tempCD = interp1(log(TABLE.VEH.matCYLINDER(:,1)),log(TABLE.VEH.matCYLINDER(:,2)),log(vecRE));
    vecCD = exp(tempCD);

	vecCD = vecCD.*(sin(valAOA).^3)+0.02;
	vecCL = vecCD.*(sin(valAOA).^2).*cos(valAOA);

    if any(STATE.VEL_MAG==0)
        vecCD(isnan(valAOA)) = 0;
        vecCL(isnan(valAOA)) = 0;
    end
    vecDRAG = vecCD.*(0.5.*AIR.density.*(STATE.VEL_MAG.^2).*(INFO.valDIAM.*valLENGTH));
    vecDRAG(vecRE==0) = 0; % If Reynolds number is 0, drag is 0
    vecLIFT  = vecCL.*(0.5.*AIR.density.*(STATE.VEL_MAG.^2).*(INFO.valDIAM.*valLENGTH));
 
    
end
    
