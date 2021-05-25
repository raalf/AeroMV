function [w_wake] = fcnWDVEVEL(fpg, valTIMESTEP, WAKE, SURF, FLAG)

% WAKE.vecWDVEHVSPN, WAKE.vecWDVEHVCRD, WAKE.vecWDVEROLL, WAKE.vecWDVEPITCH, WAKE.vecWDVEYAW, WAKE.vecWDVELESWP, ...
% WAKE.vecWDVEMCSWP, WAKE.vecWDVETESWP, WAKE.matWDVENORM, WAKE.matWVLST, WAKE.matWDVE, WAKE.valWNELE, ...
% WAKE.matWCENTER, WAKE.matWCOEFF, WAKE.vecWK

% Truncate wake, only use wake elements of interest
WAKE.vecWDVEHVSPN = WAKE.vecWDVEHVSPN(WAKE.idxTRUNC);
WAKE.vecWDVEHVCRD = WAKE.vecWDVEHVCRD(WAKE.idxTRUNC);
WAKE.vecWDVEROLL  = WAKE.vecWDVEROLL(WAKE.idxTRUNC);
WAKE.vecWDVEPITCH = WAKE.vecWDVEPITCH(WAKE.idxTRUNC);
WAKE.vecWDVEYAW   = WAKE.vecWDVEYAW(WAKE.idxTRUNC);
WAKE.vecWDVELESWP = WAKE.vecWDVELESWP(WAKE.idxTRUNC);
WAKE.vecWDVEMCSWP = WAKE.vecWDVEMCSWP(WAKE.idxTRUNC);
WAKE.vecWDVETESWP = WAKE.vecWDVETESWP(WAKE.idxTRUNC);
WAKE.matWDVENORM  = WAKE.matWDVENORM(WAKE.idxTRUNC,:);
% WAKE.matWVLST     = WAKE.matWVLST;
WAKE.matWDVE      = WAKE.matWDVE(WAKE.idxTRUNC,:);
WAKE.valWNELE     = WAKE.valWNELE - sum(~WAKE.idxTRUNC,1);
WAKE.matWCENTER   = WAKE.matWCENTER(WAKE.idxTRUNC,:);  
WAKE.matWCOEFF = WAKE.matWCOEFF(WAKE.idxTRUNC,:);      
WAKE.vecWK = WAKE.vecWK(WAKE.idxTRUNC);


% List of wDVEs we are influencing from (each one for each of the fieldpoints)
len = length(fpg(:,1));
dvenum = reshape(repmat(1:WAKE.valWNELE,len,1),[],1);

fpg = repmat(fpg,WAKE.valWNELE,1);

if FLAG.STEADY == 1
    % DVE type 1 is a regular wake DVE
    dvetype = ones(length(dvenum),1);

    % Newest row of wake DVEs have a filament at the leading edge
    newest_row = [((WAKE.valWNELE-WAKE.valWSIZE)+1):1:WAKE.valWNELE]';

    dvetype(ismember(dvenum, newest_row)) = 2;

    % Oldest row of wake DVEs are semi-infinite
    oldest_row = [1:WAKE.valWSIZE]';

    if valTIMESTEP == 1
        dvetype(ismember(dvenum, oldest_row)) = -3;
    elseif valTIMESTEP > 0
        dvetype(ismember(dvenum, oldest_row)) = 3;
    end

elseif FLAG.STEADY == 0 || 2
    % DVE type 1 is a regular wake DVE
    dvetype = zeros(length(dvenum),1);

    % Oldest row of wake DVEs are semi-infinite
    oldest_row = [1:WAKE.valWSIZE]';
    
    dvetype(ismember(dvenum, oldest_row)) = -3;
    
end

[w_ind] = fcnDVEVEL(dvenum, fpg, dvetype, WAKE.matWDVE, WAKE.matWVLST, WAKE.matWCOEFF, WAKE.vecWK, WAKE.vecWDVEHVSPN, WAKE.vecWDVEHVCRD, WAKE.vecWDVEROLL, WAKE.vecWDVEPITCH, WAKE.vecWDVEYAW, WAKE.vecWDVELESWP, WAKE.vecWDVETESWP, SURF.vecDVESYM, FLAG.GPU);

w_wake = reshape(sum(reshape(w_ind', len*3, [])',1),3,[])';
end

