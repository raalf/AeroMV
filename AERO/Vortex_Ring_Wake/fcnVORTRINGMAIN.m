function [matQ,VORTPARAM] = fcnVORTRINGMAIN(num_seg,num_ring,GEOM,COND)
% fcnVORTRINGMAIN this the main function for the Vortex Ring Wake model
%
% INPUTS:
%   GEOM.N_b        - Number of blades, size(1,1)
%   GEOM.R          - Rotor radius, size = (1,1)
%   GEOM.ROTCENTER  - Rotor centers, size (num rotors,3)
%
%   COND.V_inf      - Velocity relative to the rotor frame, size = (num rotors,3)
%   COND.RPM      - Rotor rpm, size = (num rotors,1)
%   COND.T          - Rotor thrust (N), size = (num rotor,1)
%   COND.rho        - Density (kg/m^3), size(1,1)
%
%   num_seg         - Number of vortex segements per ring, size(1,1)
%   num_ring        - Number of ring vortex
%
% OUTPUTS:
%   matQ            - Induced velocity at each rotor hub, size = (num rotors,3)
%   VORTPARAM       - Structure with general parameters of the rings

%% Procedure for this function:
%       - Calculate the circuation strength, skew angle and z-offset of the vortex rings
%       - Calculate edge locations of the vortex segments
%       - Calculate the induced velocity at the center of the rotor plane

%% Calculate the vortex paremeters (circulation, skew, z-offset)
VORTPARAM = fcnVORTPARAM(GEOM.N_b,COND.RPM,COND.V_inf,COND.T,COND.rho,GEOM.R);

% Calculate the wake geometry
VORTPARAM = fcnRINGGEOM(VORTPARAM, num_seg, num_ring, GEOM.R, GEOM.ROTCENTER,COND.V_inf);

% Calculate the induced velocit at each rotor center using Biot-Savart law
matQ = SegmentVort(VORTPARAM.matCIRC, GEOM.ROTCENTER, VORTPARAM.seg_start, VORTPARAM.seg_end);
end

