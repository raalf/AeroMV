function [ matQ ] = SegmentVort(vecGAMMA, matP, matVOR1, matVOR2, idxROTOR)
% This function calculates the induced velocity due to a vortex segment.
% This is done using Biot-Savart
%
% INPUTS
%   vecGAMMA - Circulation of each vortex segmant (associated with matVOR)
%   matP - A matrix of points of interest. Each row is associated to a POI
%   and the columns correspone to the x,y,z components of each points
%   matVOR1 - Location of left vortex segment end
%   matVOR2 - Location of right vortex segment end
%   idxROTOR - Which rotor segement belongs to which rotor. Rows are
%           associated with segment rows and value is associated to rotor
%           number
%
% OUTPUTS
%   matQ - Induced velocity due to vortex segment at POI. Each row is
%   associated to each POI. Columns associated to u,v,w velocities
%
% This function was recyled from: https://github.com/devinbarcelos/horseshoe
% with some modifications

% Changing sizes of variables to properly consider all segements impact on
% all points of interest.
num_POI = size(matP,1);
num_seg = size(matVOR1,1);
matVOR1 = repmat(matVOR1,num_POI,1);
matVOR2 = repmat(matVOR2,num_POI,1);
idxROTOR = repmat(idxROTOR,num_POI,1);
vecGAMMA = repmat(vecGAMMA,num_POI,1);
idxPOI = reshape(repmat(1:num_POI,num_seg,1),num_seg*num_POI,1);
matP = reshape(repmat(permute(matP,[2 3 1]),1,num_seg),3,num_seg*num_POI,1)';

% Ignore self induced velocities
idxSELF = idxROTOR==idxPOI;
matVOR1(idxSELF,:) = []; matVOR2(idxSELF,:)=[]; idxROTOR(idxSELF) = [];
vecGAMMA(idxSELF) = []; idxPOI(idxSELF) = []; matP(idxSELF,:) = [];

% Calcualte radius
matR1 = matVOR1 - matP; % Radius from point vortex seg end 1 to POI
matR2 = matVOR2 - matP; % Radius from point vortex seg end 2 to POI
matR0 = matR1 - matR2; % Difference between radius

matR1xR2 = cross(matR1,matR2); % Cross product between raduis

% Calculate vector magnitudes
matR1MAG = sqrt(matR1(:,1).^2+matR1(:,2).^2+matR1(:,3).^2);
matR2MAG = sqrt(matR2(:,1).^2+matR2(:,2).^2+matR2(:,3).^2);
matR1xR2MAG = sqrt(matR1xR2(:,1).^2+matR1xR2(:,2).^2+matR1xR2(:,3).^2);


% Calcualte the induced velicty at each point of interest (matP)
tempQ = (vecGAMMA/(4*pi)).*((matR1xR2)./((matR1xR2MAG).^2)).*(dot(matR0', ...
    ((matR1./matR1MAG)-(matR2./matR2MAG))'))';

% Sum the total induced velocities of all wakes at each point of interest
for i = 1:num_POI
    matQ(i,:) = sum(tempQ(idxPOI == i,:));
end

end

