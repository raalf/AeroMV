function PERF = fcnRUNVORTRING(GEOM,AIR,PERF,STATE)
%fcnRUNVORTRING calculate the rotor parameters using a vortex ring wake 
%   model. This function will iterate until thrust has converged based on
%   the interference of the vortex ring wake model.
%   This function sets up and calls fcnVORTRINGMAIN (which is located in
%   \AERO\Vortex_Ring_Wake)


% *** VALUES ARE HARD CODED IN ***
num_seg = 100; % Number of vortex segement per ring
num_ring = 200; % Number of vortex rings
convergence_criteria = 10^-4; % Congergence criteria. Max % difference

% Get initial rotor performance from lookup tables
PERF = fcnLOOKUP(GEOM, AIR, PERF, STATE);


% Setting up input structures for fcnVORTRINGMAIN
% GEOM_VORT - geometry related inputs
GEOM_VORT.ROTCENTER = GEOM.ROTOR.matLOCATION;
GEOM_VORT.N_b = GEOM.ROTOR.valNUMB;
GEOM_VORT.R = GEOM.ROTOR.vecDIAM(1)/2;

% COND - condition related inputs
COND.RPM = STATE.RPM';
COND.rho = AIR.density;
COND.V_inf = STATE.VEL_ROTOR;

% Setting up iteration loop
converged = false;
failed = 0;

% Iterate until convergences is reached or a max of 10 iterations
while ~converged
    COND.T = PERF.ROTOR.T';

    %Run fcnVORTRINGMAIN
    [matINDVEL,~] = fcnVORTRINGMAIN(num_seg,num_ring,GEOM_VORT,COND);

    % Compute the new rotor properties with the updated induced velocities
    [STATE] = fcnSTATES2AOA(STATE, GEOM,matINDVEL);
    PERF = fcnLOOKUP(GEOM, AIR, PERF, STATE);
    
    % Calculate percent difference in thrust and compare to convergence criteria
    difference = 100*abs(abs(PERF.ROTOR.T'-COND.T)./(PERF.ROTOR.T'+COND.T));
    if ~any(difference > convergence_criteria)
        converged = true;
    elseif failed == 10 % Attempt to converge for a max of 10 iterations
        disp('fcnRUNVORTRING: Unable to converge!')
        converged = true;
    else
        failed = failed + 1;
    end
end

% Print out the number of iterations to converge
fprintf('fcnRUNVORTRING took %d iteration to converge.\n',failed)