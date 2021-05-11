function [OUTP, matROTORDP, vecDELNDIST] = fcnVISCOUS_WIPP_ARCHIVED(valTIMESTEP, OUTP, COND, VISC, SURF, INPU, FLAG, WAKE, temp_visc)
% temp_visc is a flag to determine whether to use the viscous function
% within the timestepping procedure or at the end of the timestepping. 0
% will only apply it at the end and 1 will apply it within the timestepping


matROTORDP = zeros(SURF.valNELE,3);
vecDELNDIST = zeros(SURF.valNELE,1);
matINDUCEDVEL_WING = zeros(SURF.valNELE,3);
if FLAG.VISCOUS == 1
    for i = 1:INPU.valVEHICLES
        
        idxvehwing = SURF.vecDVEWING > 0 & SURF.vecDVEVEHICLE == i; %(SURF.vecDVEWING.*SURF.vecDVEVEHICLE == i) > 0;
        idxvehrotor = SURF.vecDVEROTOR > 0 & SURF.vecDVEVEHICLE == i;
        if (any(idxvehwing) && valTIMESTEP == COND.valMAXTIME) || (any(idxvehwing)&& temp_visc == 1) || (any(idxvehrotor) && any(idxvehwing) && valTIMESTEP > COND.valMAXTIME - abs(1/(min(COND.vecROTORRPM)*COND.valDELTIME/60)))
            % Compute induced velocity
            [matWUINF] = fcnINDVEL(SURF.matCENTER, valTIMESTEP, SURF, WAKE, INPU, FLAG);
             
            if COND.vecVEHVINF == 1 && FLAG.FIXEDLIFT == 1
                fixed_lift = 1;
            else
                fixed_lift = 0;
            end
            
            
            if any(idxvehrotor) || max(SURF.vecDVEWING)>1
                for d = 1:max(SURF.vecDVEWING)
                    idxDVE = SURF.vecDVEWING == d;
                    w_surf = fcnSDVEVELVISC(SURF.matCENTER(idxDVE,:), SURF, INPU, FLAG, idxDVE);
                    idxWDVE = WAKE.vecWDVESURFACE == d; % This assumes that WDVESURFACE defines wings then rotors in order
                    w_wake = fcnWDVEVELVISC(SURF.matCENTER(idxDVE,:), valTIMESTEP, WAKE, SURF, FLAG, idxWDVE);
                    
                    matINDUCEDVEL_WING(idxDVE,:) = w_surf + w_wake;
                end
            end
            
            [OUTP.vecCLv(valTIMESTEP,i), OUTP.vecCD(valTIMESTEP,i), OUTP.vecPREQ(valTIMESTEP,i), OUTP.vecLD(valTIMESTEP,i), OUTP.vecVINF(valTIMESTEP,i), OUTP.vecCMDIST, OUTP.vecCNDIST, OUTP.vecCDPDIST] = fcnVISCOUS_WING(OUTP.vecCL(valTIMESTEP), OUTP.vecCDI(valTIMESTEP), ...
                INPU.vecAREA, COND.valDENSITY, VISC.valKINV, SURF.vecDVENFREE, SURF.vecDVENIND, ...
                SURF.vecDVELFREE, SURF.vecDVELIND, SURF.vecDVESFREE, SURF.vecDVESIND, SURF.vecDVEPANEL, SURF.vecDVELE, SURF.vecDVEWING.*uint8(idxvehwing), INPU.vecN, INPU.vecM, SURF.vecDVEAREA, ...
                SURF.matCENTER, SURF.vecDVEHVCRD, VISC.cellAIRFOIL, FLAG.OLDPRINT, INPU.vecSYM, VISC.vecINTERF, SURF.vecDVEROLL, SURF.matUINF, matINDUCEDVEL_WING, SURF.matDVE, SURF.matVLST, COND.vecVEHVINF(i), fixed_lift, COND.vecVEHWEIGHT(i));
                 
        if FLAG.NACELLE == 1 && any(idxvehrotor) && any(idxvehwing)
            [OUTP] = fcnVISCOUS_NACELLE(valTIMESTEP, COND, SURF, WAKE, INPU, FLAG, VISC, OUTP);
        end
            
            
        end
        
        
%         idxvehrotor = SURF.vecDVEROTOR > 0 & SURF.vecDVEVEHICLE == i;
        if any(idxvehrotor)
            if valTIMESTEP > COND.valMAXTIME - abs(1/(min(COND.vecROTORRPM)*COND.valDELTIME/60)) % Only compute for last full rotor rotation
                % Compute induced velocity
                [matWUINF] = fcnINDVEL(SURF.matCENTER(idxvehrotor==1,:), valTIMESTEP, SURF, WAKE, INPU, FLAG);
                
                [matROTORDP(idxvehrotor,:), vecDELNDIST(idxvehrotor)] = fcnVISCOUS_ROTOR(VISC.valKINV,...
                    SURF.vecDVEHVCRD(idxvehrotor==1), INPU.vecN, INPU.vecM, SURF.vecDVELE(idxvehrotor==1), SURF.vecDVEPANEL(idxvehrotor==1), VISC.cellAIRFOIL, SURF.vecDVENFREE(idxvehrotor==1)+SURF.vecDVENIND(idxvehrotor==1), SURF.vecDVEAREA(idxvehrotor==1),SURF.matUINF(idxvehrotor==1,:), SURF.matVLST, SURF.matDVE(idxvehrotor==1,:), matWUINF, FLAG.OLDPRINT);
            end
        end
           
    end
end

end

