function [output] = fcnRUNSIMU(input)
% Run function for simulink "interpreted MATLAB fcn" block 

filename = 'AscTec_Pelican';
STATE.VEL = input(1); % m/s
STATE.AOA = input(2); % Deg
STATE.RPM = input(3:end)'; % RPM

if any(STATE.RPM==0)
    idx = find(STATE.RPM==0);
    STATE.RPM(idx) = 10;
end
% STATE.VEL = 5; % m/s
% STATE.RPM = [3000 3150 3220 3215]; % RPM
% STATE.AOA = 25; % Deg
tic
[PERF, TABLE, GEOM, AIR, STATE] = fcnMAIN(filename, STATE);
output = PERF.ROTOR(randi([1 4])).CT;
toc
end

