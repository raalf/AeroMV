function WAKE = fcnRELAXWAKE(valTIMESTEP, SURF, WAKE, COND, FLAG, INPU)

%FCNRLXWAKE Summary of this function goes here
%   Detailed explanation goes here
[ WAKE.matWDVEMP, WAKE.matWDVEMPIDX, vecWMPUP, vecWMPDN ] = fcnWDVEMP(WAKE.matWDVE, WAKE.matWVLST, WAKE.matWADJE, WAKE.valWNELE, WAKE.vecWDVESYM, WAKE.vecWDVETIP);

% Get mid-points induced velocity
[WAKE.matWDVEMPIND] = fcnINDVEL(WAKE.matWDVEMP, valTIMESTEP, SURF, WAKE, INPU, FLAG);

% Propagating wake for hover
if any(SURF.vecDVEROTOR > 0)
    maxRot = 2; 
    startVel = 6;
    valAZNUM = 1/(COND.valDELTIME*(abs(COND.vecROTORRPM(1))/60));
    if valTIMESTEP/valAZNUM <= maxRot && FLAG.HOVERWAKE == 1 && sum(COND.vecVEHVINF) == 0
        Vel = -startVel/maxRot*(valTIMESTEP/valAZNUM)+startVel;
        WAKE.matWDVEMPIND(:,3) = WAKE.matWDVEMPIND(:,3) - Vel;
    end
end

% Assemble matrices for fcnDISPLACE (vup, vnow, vdown)
[ matVUP, matVNOW, matVDOWN ] = fcnDISPMAT(WAKE.matWDVEMPIND, vecWMPUP, vecWMPDN );

% Calculate mid-point displacement
[WAKE.matWDVEMPRLX] = fcnDISPLACE(matVUP, matVNOW, matVDOWN, WAKE.matWDVEMP, COND.valDELTIME);

% update WAKE.matWCENTER
WAKE.matWCENTER = WAKE.matWDVEMPRLX(WAKE.matWDVEMPIDX(:,1),:)+(0.5*(WAKE.matWDVEMPRLX(WAKE.matWDVEMPIDX(:,2),:)-WAKE.matWDVEMPRLX(WAKE.matWDVEMPIDX(:,1),:)));

% Calculate the leading edge mid-points by refering to upstream dves
WAKE.matWDVELEMPIDX = flipud(reshape(1:WAKE.valWNELE,WAKE.valWSIZE,[])');
WAKE.matWDVELEMP = nan(WAKE.valWNELE,3);
for wakerow = 1:valTIMESTEP
    if wakerow == 1 %|| wakerow == valTIMESTEP % freshest row of wake, closest to wing TE
        WAKE.matWDVELEMP(WAKE.matWDVELEMPIDX(wakerow,:),(1:3)) = permute(mean(reshape(WAKE.matWVLST(WAKE.matWDVE(WAKE.matWDVELEMPIDX(wakerow,:),[1,2]),:)',3,[],2),3),[3 2 1]);
    else % rest of the wake dves
        prevLE = WAKE.matWDVELEMP(WAKE.matWDVELEMPIDX(wakerow-1,:),(1:3));
        prevXO = WAKE.matWCENTER(WAKE.matWDVELEMPIDX(wakerow-1,:),(1:3));
        WAKE.matWDVELEMP(WAKE.matWDVELEMPIDX(wakerow,:),(1:3)) = prevLE + 2.*(prevXO - prevLE);
    end
end

% half chord vector, leading edge midpoint -> control point
crdvec = WAKE.matWCENTER - WAKE.matWDVELEMP;
% half span vector, control point -> right edge midpoint
spnvec = WAKE.matWDVEMPRLX(WAKE.matWDVEMPIDX(:,2),:) - WAKE.matWCENTER;

% fix oldest wake
% semiinfvec = WAKE.matWCENTER(WAKE.matWDVELEMPIDX(end-1,:),:)-WAKE.matWDVELEMP(WAKE.matWDVELEMPIDX(end-1,:),:);
% WAKE.matWCENTER(WAKE.matWDVELEMPIDX(end,:),:) = WAKE.matWDVELEMP(WAKE.matWDVELEMPIDX(end,:),:)+semiinfvec;

% xsi0 = repmat(sqrt(semiinfvec(:,1).^2+semiinfvec(:,2).^2+semiinfvec(:,3).^2),1,3);
% WAKE.matWCENTER(WAKE.matWDVELEMPIDX(end,:),:) = WAKE.matWCENTER(WAKE.matWDVELEMPIDX(end-1,:),:)+semiinfvec+SURF.matUINF(SURF.vecDVETE == 3).*xsi0;
% crdvec(WAKE.matWDVELEMPIDX(end,:),:) = crdvec(WAKE.matWDVELEMPIDX(end-1,:),:);
% spnvec(WAKE.matWDVELEMPIDX(end,:),:) = spnvec(WAKE.matWDVELEMPIDX(end-1,:),:);


% Calculate four corner point by adding vectors to control point coordinates
WP1 = WAKE.matWCENTER - crdvec - spnvec;
WP2 = WAKE.matWCENTER - crdvec + spnvec;
WP3 = WAKE.matWCENTER + crdvec + spnvec;
WP4 = WAKE.matWCENTER + crdvec - spnvec;

%% Recalculating oldest row 4 corner points
% This overwrites the WP1-WP4 points of oldest wake elements
oldestwake = reshape(WAKE.matWDVELEMPIDX(end,:),[],1);
secondoldestwake = reshape(WAKE.matWDVELEMPIDX(end-1,:),[],1);

translationp1 = WP1(oldestwake,:)-WP4(secondoldestwake,:);
translationp2 = WP2(oldestwake,:)-WP3(secondoldestwake,:);

WP1(oldestwake,:) = WP4(secondoldestwake,:);
WP2(oldestwake,:) = WP3(secondoldestwake,:);
timesteptranslate = SURF.matUINF(SURF.vecDVETE == 3,:)*COND.valDELTIME;


% Condition: if there is are rotors, do not prescibe the last row of wake
% elements in the freestream, instead base it off the delta between the
% TE to LE angle of the second last row of wake elements

WP4_rotor = WP4; WP4_wing = WP4;
WP3_rotor = WP3; WP3_wing = WP3;
temp = unique(SURF.vecDVESURFACE(SURF.vecDVEWING>0)); % Identify which surfaces are wings
idx = sum(repmat(WAKE.vecWDVESURFACE,1,length(temp)) == repmat(temp',length(WAKE.vecWDVESURFACE),1),2) == 1;
WP4_wing(oldestwake,:) = WP1(oldestwake,:)+timesteptranslate;
WP3_wing(oldestwake,:) = WP2(oldestwake,:)+timesteptranslate;

WP4_rotor(oldestwake,:) = WP4(oldestwake,:)+translationp1;
WP3_rotor(oldestwake,:) = WP3(oldestwake,:)+translationp2;

% Add wing or rotor specific last row to the WP4 and WP3
WP4(idx,:) = WP4_wing(idx,:); WP3(idx,:) = WP3_wing(idx,:);
WP4(~idx,:) = WP4_rotor(~idx,:); WP3(~idx,:) = WP3_rotor(~idx,:);

WAKE.matWCENTER(oldestwake,:) = (WP1(oldestwake,:)+WP2(oldestwake,:)+WP3(oldestwake,:)+WP4(oldestwake,:))./4;

%%
% update relax wake dves
[WAKE.vecWDVEHVSPN, WAKE.vecWDVEHVCRD, WAKE.vecWDVEROLL, WAKE.vecWDVEPITCH, WAKE.vecWDVEYAW,...
    WAKE.vecWDVELESWP, WAKE.vecWDVEMCSWP, WAKE.vecWDVETESWP, WAKE.vecWDVEAREA, WAKE.matWDVENORM, ...
    WAKE.matWVLST, WAKE.matWDVE, ~, ~] = fcnDVECORNER2PARAM( WAKE.matWCENTER, WP1, WP2, WP3, WP4 );

% For singularity factor updating, each row of wake elements needs to be seen as its own "wing"
% so we are adding on to the vecWDVEWING by timestep number to create the right form to pass into
% the surface singularity factor function
tswing = double(WAKE.vecWDVESURFACE) + reshape(repmat([0:double(max(WAKE.vecWDVESURFACE)):((WAKE.valWNELE/WAKE.valWSIZE)-1)*double(max(WAKE.vecWDVESURFACE))], WAKE.valWSIZE,1),[],1);

% Updating wake singularity factor
[WAKE.vecWK] = fcnSINGFCT(WAKE.valWNELE, tswing, WAKE.vecWDVETIP, WAKE.vecWDVEHVSPN);

end

