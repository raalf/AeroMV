function [OUTP, PERF, TABLE, GEOM, AIR, STATE] = fcnMAIN(TABLE, GEOM, AIR, STATE, idxAERO, OVERWRITE)
%% This is the main function
% The general procedure of the main is:
%	Read geometric and state inputs
%   Convert states to angle of attack for each component/rotor
%	Calculate the vehicle component aerodynamics
%	Calculate the rotor aerodynamics
%	Calculate total forces and moments acting on vehicle
%	Apply flight dynamics model to calculate the new vehicles states
%
% INPUTS:
%   filename - String of input filename
%   STATE - Structure of vehicle states which includes:
%         STATE.VEL: (x,y,z) Velocity in inertial frame (m/s)
%         STATE.POS: (x,y,z) Position in inertial frame (m)
%         STATE.EULER: (phi, theta, psi) Euler angles (rad)
%         STATE.BODY_RATES: (p,q,r) Rotational accelerations in inertial
%         frame (rad/s)
%         STATE.RPM: Individual rotor speeds (Rev/Min)
%   idxAERO - an identifier of which rotor aerodynamics model to use.
%               = 1: Lookup tables
%               = 2: Run BEMT
%               = 3: Lookup tables with vortex ring model
%               = 4: Simple K*Omega^2 approach
%               = 5: Run BEMT with radial velocities from flight dynamics
%               = 6: Runs VAP3.5 for each rotor individually

%% Retrieve Input Vehicle Geometry (moved to outside of fcnMAIN for computational speed)
% [TABLE, GEOM, AIR] = fcnINPUT(filename);
idxVEHPERF = 1; % Toggle for calculating vehicle component forces

%% Overwrite variables if neccessary
if exist('OVERWRITE','var')
    if ~isempty(OVERWRITE)
        [AIR,GEOM,TABLE,STATE] = fcnOVERWRITE(OVERWRITE,AIR,GEOM,TABLE,STATE);
    end
end

%% Re-calculate the geometry to be relative to CG
[GEOM] = fcnRELATIVE2CG(GEOM);

%% Convert Input States to Angle of Attack
[STATE] = fcnSTATES2AOA(STATE, GEOM);

%% Vehicle Body Force Buildup
if (idxVEHPERF ~= 0)
    [PERF, TABLE, GEOM] = fcnVEHPERF(AIR, TABLE, GEOM, STATE);
else
    PERF.VEHOFF = 1;
end

%% Rotor Aerodynamics
if (idxAERO == 1)
    % Lookup Table
    PERF = fcnLOOKUP(GEOM, AIR, PERF, STATE);
elseif (idxAERO==2)
    % BEMT Module
    PERF = fcnRUNBEMT(GEOM, AIR, PERF, STATE);
elseif (idxAERO==3)
    % Vortex ring wake model
    PERF = fcnRUNVORTRING(GEOM,AIR,PERF,STATE);
elseif (idxAERO==4)
    PERF = fcnKOMEGA2(GEOM,AIR,PERF,STATE);
elseif (idxAERO==5)
    % BEMT Module with velocity distributions
    PERF = fcnRUNBEMT_VELDIST(GEOM, AIR, PERF, STATE);
elseif (idxAERO==6)
     PERF = fcnRUNVAPSINGLE(GEOM,AIR,PERF,STATE);
end

% Apply ground effect
PERF = fcnGRDEFF(GEOM,PERF,STATE);

%% Force and Moment Transformations
[OUTP] = fcnFORCES(PERF, GEOM, STATE, idxVEHPERF);

%% Flight Dynamics Model
[OUTP] = fcnFLIGHTDYN(GEOM, STATE, OUTP);
