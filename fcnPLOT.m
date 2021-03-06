function [] = fcnPLOT(filename,fignum)
% fcnPLOT This function plots the geometry given by filename
%
% INPUTS:
%   filename        - A string with filename. File should be located in
%                   TABLES folder
%   fignum          - The figure number wanted
%
%
% Example: fcnPLOT('AscTec_Pelican',1)


%% Setup plotting
% Read in geometry from input file
[~, GEOM, ~] = fcnINPUT(filename);

% Create and setup figure
figure(fignum)
clf(fignum)
axis equal
grid on
box on
xlabel('X-Dir')
ylabel('Y-Dir')
zlabel('Z-Dir')

%% Plot rotors
for i = 1:size(GEOM.ROTOR.matLOCATION,1)
    C = GEOM.ROTOR.matLOCATION(i,:) ;   % center of circle
    R = GEOM.ROTOR.vecDIAM(i)/2;    % Radius of circle
    teta=0:0.01:2*pi ;
    x=C(1)+R*cos(teta);
    y=C(2)+R*sin(teta) ;
    z = C(3)+zeros(size(x)) ;
    patch(x,y,z,'b')
    hold on
    plot3(C(1),C(2),C(3),'*r')
    str = sprintf('%d',i);
    text(C(1),C(2),C(3),str,'Color','k','FontSize',40);
end

%% Plot individual components
% BODY
GEOM.VEH.idxBODY = fcnCOMPCHECK(GEOM.VEH,'BODY');
if GEOM.VEH.idxBODY
    fcnPLTCOMP(GEOM.VEH.BODY,fignum)
end

% ARMS
GEOM.VEH.idxARM = fcnCOMPCHECK(GEOM.VEH,'ARM');
if GEOM.VEH.idxARM
    fcnPLTCOMP(GEOM.VEH.ARM,fignum)
end

% MOTORS
GEOM.VEH.idxMOTOR = fcnCOMPCHECK(GEOM.VEH,'MOTOR');
if GEOM.VEH.idxMOTOR
    fcnPLTCOMP(GEOM.VEH.MOTOR,fignum)
end

% LEGS
GEOM.VEH.idxLEG = fcnCOMPCHECK(GEOM.VEH,'LEG');
if GEOM.VEH.idxLEG
    fcnPLTCOMP(GEOM.VEH.LEG,fignum)
end

% PAYLOAD
GEOM.VEH.idxPAYLOAD = fcnCOMPCHECK(GEOM.VEH,'PAYLOAD');
if GEOM.VEH.idxPAYLOAD
    fcnPLTCOMP(GEOM.VEH.PAYLOAD,fignum)
end

GEOM.VEH.idxOTHER = fcnCOMPCHECK(GEOM.VEH,'OTHER');
if GEOM.VEH.idxOTHER
    fcnPLTCOMP(GEOM.VEH.OTHER,fignum)
end

%% Add point for CG
figure(fignum)
hold on
scatter3(GEOM.VEH.vecCG(:,1),GEOM.VEH.vecCG(:,2),GEOM.VEH.vecCG(:,3),'kp','linewidth',3)
hold off