function  [OUTP, COND, INPU, FLAG, MISC, SURF, VEHI, VISC, WAKE] = fcnVAPTIMESTEP(FLAG, COND, VISC, INPU, VEHI, WAKE, SURF, OUTP,MISC, valBEGINTIME)
%fcnVAPTIMESTEP This run the time stepping procedure for a case that has
%already been initizlized

if ~exist("valBEGINTIME","var")
    valBEGINTIME = 1;
end
if ~(COND.valMAXTIME==size(OUTP.vecCL,1))
    [OUTP] = fcnREINIT(OUTP,COND.valMAXTIME,INPU.valVEHICLES);
end

for valTIMESTEP = valBEGINTIME:COND.valMAXTIME
    %% Timestep to solution
    %   Move wing
    %   Generate new wake elements
    %   Create and solve WD-Matrix for new elements
    %   Solve wing D-Matrix with wake-induced velocities
    %   Solve entire WD-Matrix
    %   Relaxation procedure (Relax, create W-Matrix and W-Resultant, solve W-Matrix)
    %   Calculate surface normal forces
    %   Calculate DVE normal forces
    %   Calculate induced drag
    %   Calculate cn, cl, cy, cdi
    %   Calculate viscous effects

    %% Moving the vehicles

    % Bend wing if applicable, else move wing normally
    [SURF, INPU, MISC, VISC] = fcnMOVESURFACE(INPU, VEHI, MISC, COND, SURF, VISC);

    if max(SURF.vecDVEROTOR) > 0
        SURF.matD = fcnKINCON(SURF.matD(1:(size(SURF.matD,1)*(2/3)),:), SURF, INPU, FLAG);
    end

    %% Generating new wake elements
    [INPU, COND, MISC, VISC, WAKE, VEHI, SURF] = fcnCREATEWAKEROW(FLAG, INPU, COND, MISC, VISC, WAKE, VEHI, SURF);

    if FLAG.PREVIEW ~= 1
        %% Creating and solving WD-Matrix for latest row of wake elements
        % We need to grab from WAKE.matWADJE only the values we need for this latest row of wake DVEs
        idx = sparse(sum(ismember(WAKE.matWADJE,[((WAKE.valWNELE - WAKE.valWSIZE) + 1):WAKE.valWNELE]'),2)>0 & (WAKE.matWADJE(:,2) == 4 | WAKE.matWADJE(:,2) == 2));
        temp_WADJE = [WAKE.matWADJE(idx,1) - (valTIMESTEP-1)*WAKE.valWSIZE WAKE.matWADJE(idx,2) WAKE.matWADJE(idx,3) - (valTIMESTEP-1)*WAKE.valWSIZE];

        [matWD, WAKE.vecWR] = fcnWDWAKE([1:WAKE.valWSIZE]', temp_WADJE, WAKE.vecWDVEHVSPN(end-WAKE.valWSIZE+1:end), WAKE.vecWDVESYM(end-WAKE.valWSIZE+1:end), WAKE.vecWDVETIP(end-WAKE.valWSIZE+1:end), WAKE.vecWKGAM(end-WAKE.valWSIZE+1:end), INPU.vecN);
        [WAKE.matWCOEFF(end-WAKE.valWSIZE+1:end,:)] = fcnSOLVEWD(matWD, WAKE.vecWR, WAKE.valWSIZE, WAKE.vecWKGAM(end-WAKE.valWSIZE+1:end), WAKE.vecWDVEHVSPN(end-WAKE.valWSIZE+1:end));

        %% Truncate wake
        % Wake truncation will simply use the wake data
        % Intial assignment of wake variables specific to truncating
        WAKE.idxTRUNC = ones(WAKE.valWSIZE*valTIMESTEP,1) == 1;
        WAKE.idxTRUNCADJE = ones(size(WAKE.matWADJE,1),1) == 1;

        if FLAG.TRUNCATE && INPU.valTIMETRUNC < valTIMESTEP
            % Create idx for which wake elements must be included
            WAKE.idxTRUNC(1:(valTIMESTEP-INPU.valTIMETRUNC)*WAKE.valWSIZE) = 0;
            % Create a new ADJE matrix with only the dves of interest
            WAKE.idxTRUNCADJE = ismember(WAKE.matWADJE(:,1),find(WAKE.idxTRUNC))>0 & ismember(WAKE.matWADJE(:,3),find(WAKE.idxTRUNC))>0; % & (WAKE.matWADJE(:,2) == 4 | WAKE.matWADJE(:,2) == 2));
            WAKE.idxTRUNCADJE = [WAKE.matWADJE(WAKE.idxTRUNCADJE,1)-sum(~WAKE.idxTRUNC),WAKE.matWADJE(WAKE.idxTRUNCADJE,2),WAKE.matWADJE(WAKE.idxTRUNCADJE,3)-sum(~WAKE.idxTRUNC),WAKE.matWADJE(WAKE.idxTRUNCADJE,4)];
        else
            WAKE.idxTRUNCADJE = WAKE.matWADJE;
        end


        %% Rebuilding and solving wing resultant
        [vecR] = fcnRWING(valTIMESTEP, SURF, WAKE, FLAG);
        [SURF.matCOEFF] = fcnSOLVED(SURF.matD, vecR, SURF.valNELE);

        %% Creating and solving WD-Matrix
        %         [1:WAKE.valWNELE]'
        %         [1:WAKE.valWNELE-sum(WAKE.idxTRUNC==0)]'
        [matWD, WAKE.vecWR] = fcnWDWAKE([1:WAKE.valWNELE-sum(WAKE.idxTRUNC==0)]', WAKE.idxTRUNCADJE, WAKE.vecWDVEHVSPN(WAKE.idxTRUNC), WAKE.vecWDVESYM(WAKE.idxTRUNC), WAKE.vecWDVETIP(WAKE.idxTRUNC), WAKE.vecWKGAM(WAKE.idxTRUNC), INPU.vecN);
        [WAKE.matWCOEFF(WAKE.idxTRUNC,:)] = fcnSOLVEWD(matWD, WAKE.vecWR, WAKE.valWNELE-sum(WAKE.idxTRUNC==0), WAKE.vecWKGAM(WAKE.idxTRUNC), WAKE.vecWDVEHVSPN(WAKE.idxTRUNC));

        %% Relaxing wake
        if valTIMESTEP > 2 && FLAG.RELAX == 1
            old_span = WAKE.vecWDVEHVSPN;
            WAKE = fcnRELAXWAKE(valTIMESTEP, SURF, WAKE, COND, FLAG, INPU);
            WAKE.matWCOEFF(:,2:3) = WAKE.matWCOEFF(:,2:3).*[old_span./WAKE.vecWDVEHVSPN (old_span./WAKE.vecWDVEHVSPN).^2];
        end

        %% Forces
        if valTIMESTEP >= COND.valSTARTFORCES
            [INPU, COND, MISC, VISC, WAKE, VEHI, SURF, OUTP] = fcnFORCES(valTIMESTEP, FLAG, INPU, COND, MISC, VISC, WAKE, VEHI, SURF, OUTP);
        end

        if FLAG.SAVETIMESTEP == 1
            save([MISC.timestep_folder, 'timestep_', num2str(valTIMESTEP), '.mat'], 'valTIMESTEP','INPU','COND','MISC','WAKE','VEHI','SURF','OUTP','FLAG','VISC');
        end
    end

    %% Post-timestep outputs
    if FLAG.OLDPRINT == 1
        fcnPRINTOUT(FLAG.OLDPRINT, valTIMESTEP, INPU.valVEHICLES, OUTP.vecCL, OUTP.vecCDI, OUTP.vecCT, MISC.vecROTORJ, VEHI.vecROTORVEH, 1)
    end
    if FLAG.PRINT == 1
        if isnan(OUTP.vecCT(valTIMESTEP))
            fprintf('%d, ',valTIMESTEP)
        else
            fprintf('%d: CT = %0.4f, ',valTIMESTEP,OUTP.vecCT(valTIMESTEP))
            if (valTIMESTEP/5) == floor(valTIMESTEP/5)
                fprintf('\n')
            end
        end
    end

    if FLAG.GIF == 1 % Creating GIF (output to GIF/ folder by default)
        fcnGIF(valTIMESTEP, FLAG, SURF, VISC, WAKE, COND, INPU, 1)
    end
end

[OUTP] = fcnOUTPUT(COND, FLAG, INPU, SURF, OUTP, valTIMESTEP);

if FLAG.OLDPRINT == 1 && FLAG.PREVIEW == 0
    fprintf('VISCOUS CORRECTIONS => CLv = %0.4f \tCD = %0.4f \n', OUTP.vecCLv(end,:), OUTP.vecCD(end,:))
    fprintf('\n');
end

%% Plotting
if FLAG.PLOT == 1
    fcnPLOTPKG(valTIMESTEP, FLAG, SURF, VISC, WAKE, COND, INPU)
end