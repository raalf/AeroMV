function PERF = fcnRUNVORTRING(GEOM,AIR,PERF,STATE)

% FOLDER_ADDRESS = pwd;

num_seg = 100;
num_ring = 200;
convergence_criteria = 10^-4;

PERF = fcnLOOKUP(GEOM, AIR, PERF, STATE);

GEOM_VORT.ROTCENTER = GEOM.ROTOR.matLOCATION;
GEOM_VORT.N_b = GEOM.ROTOR.valNUMB;
GEOM_VORT.R = GEOM.ROTOR.vecDIAM(1)/2;

COND.RPM = STATE.RPM';
COND.rho = AIR.density;
COND.V_inf = STATE.VEL_ROTOR;

converged = false;
failed = 0;
while ~converged
    COND.T = PERF.ROTOR.T';
%     cd(strcat(FOLDER_ADDRESS,'\AERO\Vortex_Ring_Wake'))
    [matINDVEL,~] = fcnVORTRINGMAIN(num_seg,num_ring,GEOM_VORT,COND);
%     cd(FOLDER_ADDRESS)
    
    [STATE] = fcnSTATES2AOA(STATE, GEOM,matINDVEL);
    PERF = fcnLOOKUP(GEOM, AIR, PERF, STATE);
    
    difference = 100*abs(abs(PERF.ROTOR.T'-COND.T)./(PERF.ROTOR.T'+COND.T));
    if ~any(difference > convergence_criteria)
        converged = true;
    elseif failed == 10
        disp('fcnRUNVORTRING: Unable to converge!')
        converged = true;
    else
        failed = failed + 1;
    end
end

fprintf('fcnRUNVORTRING took %d iteration to converge.\n',failed)

