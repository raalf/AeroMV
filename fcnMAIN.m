function [OUTP, PERF, TABLE, GEOM, AIR, STATE] = fcnMAIN(filename, STATE)
%% This is the main function
% The general procedure of the main is:
%	Read geometric and state inputs
%	Calculate the vehicle aerodynamics
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

idxAERO = 1;
%% Retrieve input vehicle geometry
[TABLE, GEOM, AIR] = fcnINPUT(filename);

%% Convert Input States
[STATE] = fcnSTATES2AOA(STATE, GEOM);

%% Vehicle Body Force Buildup
[PERF, TABLE, GEOM] = fcnVEHPERF(AIR, TABLE, GEOM, STATE);

%% Rotor Aerodynamics
if (idxAERO == 1)
    % Lookup Table
    PERF = fcnLOOKUP(GEOM, AIR, PERF, STATE);
elseif (idxAERO==2)
    % BEMT Module
    PERF = fcnRUNBEMT(GEOM, AIR, PERF, STATE);
end

%% Force and Moment Transformations
[OUTP] = fcnFORCES(PERF, GEOM, STATE);

%% Flight Dynamics Model
[OUTP] = fcnFLIGHTDYN(GEOM, STATE, OUTP);
