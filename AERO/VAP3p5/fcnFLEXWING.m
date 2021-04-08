function [COND, INPU, OUTP, MISC, SURF, valGUSTTIME] = fcnFLEXWING(INPU, COND, SURF, OUTP, FLAG, MISC, valGUSTTIME, valTIMESTEP)

% [INPU, SURF] = fcnSTRUCTDIST(INPU, SURF);

SURF.matCENTER_old = SURF.matCENTER;

% Applies gust after aeroelastic convergence
% Michael A. D. Melville, Denver, CO, 80218
if valGUSTTIME > 1
        
    for tempTIME = 1:COND.valSTAGGERSTEPS
        [OUTP] = fcnELASTICWING_STAGGER2(OUTP, INPU, SURF, COND, valTIMESTEP, tempTIME);
    end

    OUTP.matDEF_old = OUTP.matDEF(end-1:end,:);
    OUTP.matTWIST_old = OUTP.matTWIST(end-1:end,:);
    
% Runs structure code until static aeroleastic convergence
else
    
    n = 5000;
    
    for tempTIME = 1:n
        
        [OUTP] = fcnELASTICWING_STAGGER(OUTP, INPU, SURF, COND, valTIMESTEP, tempTIME);
    
    end
    
    % Error checking for unstable solution
    if any(isnan(OUTP.vecDEF) == 1)
        fprintf('\nUnstable structure solution. Reduce time step size!\n\n')
    end
    
    OUTP.matDEF_old = OUTP.matDEF;
    OUTP.matTWIST_old = OUTP.matTWIST;
    
end

[SURF, MISC, COND] = fcnMOVEFLEXWING(COND, SURF, OUTP, INPU, MISC, valTIMESTEP);

[ SURF.vecDVEHVSPN, SURF.vecDVEHVCRD, SURF.vecDVEROLL, SURF.vecDVEPITCH, SURF.vecDVEYAW,...
    SURF.vecDVELESWP, SURF.vecDVEMCSWP, SURF.vecDVETESWP, SURF.vecDVEAREA, SURF.matDVENORM, SURF.matVLST, SURF.matDVE, SURF.matCENTER, MISC.matNEWWAKE ] ...
    = fcnVLST2DVEPARAM_NEW(SURF.matNPDVE, SURF.matNPVLST, MISC.matNEWWAKE, SURF.vecDVETE);

% Determine % relative change between aeroelastic timesteps
tol_def = (100*abs(OUTP.matDEFGLOB(valTIMESTEP,end)-OUTP.matDEFGLOB(valTIMESTEP-COND.valSTIFFSTEPS,end))/abs(OUTP.matDEFGLOB(valTIMESTEP-COND.valSTIFFSTEPS,end)));
tol_twist = (100*abs(OUTP.matTWISTGLOB(valTIMESTEP,end)-OUTP.matTWISTGLOB(valTIMESTEP-COND.valSTIFFSTEPS,end))/abs(OUTP.matTWISTGLOB(valTIMESTEP-COND.valSTIFFSTEPS,end)));

% Add in gust velocities to SURF.matUINF if convergence tolerance is met
if (tol_def < 2 && tol_twist < 2) || valGUSTTIME > 1
    
    if valGUSTTIME == 1
        fprintf('\nStatic convergence reached. Beginning gust profile.\n\n')
        COND.valGUSTSTART = valTIMESTEP+5; % Start gust 5 time steps after convergence has been reached
        COND.valDELTIME_old = COND.valDELTIME;
    end
    
    if valGUSTTIME <= 1
        
        [SURF.matUINF] = fcnFLEXUINF(SURF.matCENTER_old, SURF.matCENTER, COND.valDELTIME, COND.valSTAGGERSTEPS);
        COND.valDELTIME = COND.valSTAGGERSTEPS*COND.valSDELTIME;
        [SURF.matUINF, SURF.gust_vel_old] = fcnGUSTWING(SURF.matUINF,COND.valGUSTAMP,COND.valGUSTL,FLAG.GUSTMODE,COND.valDELTIME_old,COND.vecVEHVINF,COND.valGUSTSTART,SURF.matCENTER,SURF.gust_vel_old);
        valGUSTTIME = valGUSTTIME + 1;
        
    else
        
        COND.valDELTIME = COND.valSTAGGERSTEPS*COND.valSDELTIME;
        
        % Add elastic velocities as well as gust velocity
        [SURF.matUINF] = fcnFLEXUINF(SURF.matCENTER_old, SURF.matCENTER, COND.valDELTIME, COND.valSTAGGERSTEPS);
        [SURF.matUINF, SURF.gust_vel_old] = fcnGUSTWING(SURF.matUINF,COND.valGUSTAMP,COND.valGUSTL,FLAG.GUSTMODE,COND.valDELTIME_old,COND.vecVEHVINF,COND.valGUSTSTART,SURF.matCENTER,SURF.gust_vel_old);
        valGUSTTIME = valGUSTTIME + 1;
        
    end
    
end

end