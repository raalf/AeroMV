function [SURF, MISC, COND] = fcnMOVEFLEXWING(COND, SURF, OUTP, INPU, MISC, valTIMESTEP)
% MISC.matNEWWAKE, MISC.matNPNEWWAKE,
% This function determines the velocities with which the DVEs are moved
% based on the deflection and twist of the wing. The corresponding
% translations are then computed of the DVE vertices and control points.

[ledves, ~, ~] = find(SURF.vecDVELE > 0);

% Span of each spanwise set of DVEs
vecDVESPAN = 2*SURF.vecDVEHVSPN(ledves)';

% Calculate cartesian velocity of DVE edges

temp = [linterp(SURF.matCENTER(ledves,2),SURF.matUINF(ledves,1),SURF.vecSPANDIST(2:end-1))',...
    linterp(SURF.matCENTER(ledves,2),SURF.matUINF(ledves,2),SURF.vecSPANDIST(2:end-1))',...
    linterp(SURF.matCENTER(ledves,2),SURF.matUINF(ledves,3),SURF.vecSPANDIST(2:end-1))'];

% Extrapolate velocity at wing tips
uinf_root = ((temp(1,:) - SURF.matUINF(ledves(1),:))./(SURF.vecSPANDIST(2)-SURF.matCENTER(ledves(1),2))).*(SURF.vecSPANDIST(1)-SURF.matCENTER(ledves(1),2))+SURF.matUINF(ledves(1),:);
uinf_tip = ((temp(end,:) - SURF.matUINF(ledves(end),:))./(SURF.vecSPANDIST(end-1)-SURF.matCENTER(ledves(end),2))).*(SURF.vecSPANDIST(end)-SURF.matCENTER(ledves(end),2))+SURF.matUINF(ledves(end),:);

matUINF_edge = [uinf_root; temp; uinf_tip];

del_twist = ((OUTP.matTWISTGLOB(valTIMESTEP,:) - OUTP.matTWISTGLOB(valTIMESTEP-1,:)));
omega = ((OUTP.matTWISTGLOB(valTIMESTEP,:) - OUTP.matTWISTGLOB(valTIMESTEP-1,:)))./COND.valDELTIME;
vecXVEL = matUINF_edge(:,1);
vecYVEL = matUINF_edge(:,2) + [0, (OUTP.matDEFGLOB(valTIMESTEP,2:end)-OUTP.matDEFGLOB(valTIMESTEP-1,2:end))./...
    (COND.valDELTIME.*tan(repmat(pi/2,1,size(OUTP.matSLOPE,2))-(OUTP.matSLOPE(valTIMESTEP,:)-OUTP.matSLOPE(valTIMESTEP-1,:))./2))]';
vecZVEL = matUINF_edge(:,3) + ((OUTP.matDEFGLOB(valTIMESTEP,:) - OUTP.matDEFGLOB(valTIMESTEP-1,:))./COND.valDELTIME)';

% Determine DVEs in each spanwise station
[matROWS] = fcnDVEROW(ledves, SURF, INPU);

%% Determining displacement distances

% Determine vertices that need to be moved at each spanwise station

% All left LE and TE points to move
temp_leftV = [SURF.matNPDVE(matROWS,1),SURF.matNPDVE(matROWS,4)];
temp_leftV = reshape(temp_leftV,sum(INPU.vecN,1),[]);

[move_row,~] = find(temp_leftV); % Vector correspond to which index of deflection velocity matrix should be used for each element

% Allocate space for translation matrices
translateNTVLST = zeros(size(SURF.matNPVLST,1),3);
temp_translate = zeros(size(SURF.matNPVLST,1),3);

temp_r = [sqrt(sum(SURF.matSCLST.^2,2)), zeros(length(SURF.matSCLST(:,1)),2)]; % Distance between vertex and shear center

xz_sign = sign(SURF.matSCLST(:,1)); % Determines whether positive or negative contribution to X and Z velocity for each vertex

% Perform a linear interpolation/extrapolation to determine the pitch of
% the DVE's at their respective left and right edges. This is used to
% determine the initial orientation of the DVE before applying the twist
% caused by elastic deformation
tempSPANDIST = SURF.matCENTER(ledves,2); % Y coordinate of DVE mid-point (point where DVEPITCH is applied) --> used as "x" term for linear interpolation

vecEDGEPITCH = SURF.vecDVEPITCH(ledves); % DVE pitch along span --> used as "y" term for linear interpolation

% Some setup of work to be able to perform linear interpolation without a
% for loop
tempSPANDIST = repmat(tempSPANDIST', size(tempSPANDIST,1),1);

tempSPANDIST = triu(tempSPANDIST);

vecEDGEPITCH = repmat(vecEDGEPITCH', size(vecEDGEPITCH,1),1);

vecEDGEPITCH = triu(vecEDGEPITCH);

vecEDGEPITCH = ((SURF.vecSPANDIST(2:(end-1))' - tempSPANDIST(1,1:(end-1)))./(tempSPANDIST(2,2:end)-...
    tempSPANDIST(1,1:(end-1)))).*(vecEDGEPITCH(2,2:end)-vecEDGEPITCH(1,1:(end-1))) + vecEDGEPITCH(1,1:(end-1)); % Linear interpolation

% Adding in root and tip values using a linear extrapolation
pitch_root = vecEDGEPITCH(1,1) - (tempSPANDIST(1,1) - SURF.vecSPANDIST(1)).*(vecEDGEPITCH(1,2)-vecEDGEPITCH(1,1))./(tempSPANDIST(1,2)-tempSPANDIST(1,1));

pitch_tip = vecEDGEPITCH(1,end) + (SURF.vecSPANDIST(end) - tempSPANDIST(1,end)).*(vecEDGEPITCH(1,end)-vecEDGEPITCH(1,end-1))./(tempSPANDIST(1,end)-tempSPANDIST(1,end-1));

vecEDGEPITCH = [pitch_root, vecEDGEPITCH, pitch_tip];

% ======================== Left Edge Displacements ========================
% Translate left edge vertices due to twist
twistXDIST = -temp_r(temp_leftV,1).*cos(del_twist(move_row)+vecEDGEPITCH(move_row))' + ...
    temp_r(temp_leftV,1).*cos(vecEDGEPITCH(move_row))'; % X component of twist 
twistZDIST = temp_r(temp_leftV,1).*sin(del_twist(move_row)+vecEDGEPITCH(move_row))' -  ...
    temp_r(temp_leftV,1).*sin(vecEDGEPITCH(move_row))'; % Z component of twist

v_rot = temp_r(temp_leftV,1).*omega(move_row)';

% Assign twist displacement to translation matrix
temp_translate(temp_leftV,1) = twistXDIST;
temp_translate(temp_leftV,3) = twistZDIST;

test = zeros(size(SURF.matNPVLST,1),3);
test(temp_leftV,1) = v_rot.*sin(OUTP.matTWISTGLOB(valTIMESTEP,move_row)+vecEDGEPITCH(move_row))'.*COND.valDELTIME;
test(temp_leftV,3) = v_rot.*cos(OUTP.matTWISTGLOB(valTIMESTEP,move_row)+vecEDGEPITCH(move_row))'.*COND.valDELTIME;

% Translate left edge vertices due to freestream and bending
translateNTVLST(temp_leftV,1) = COND.valDELTIME.*vecXVEL(move_row);
translateNTVLST(temp_leftV,2) = COND.valDELTIME.*vecYVEL(move_row);
translateNTVLST(temp_leftV,3) = -1*COND.valDELTIME.*vecZVEL(move_row);

% ======================== Right Edge Displacements =======================
% All right LE and TE points to move
temp_rightV = [SURF.matNPDVE(matROWS,2), SURF.matNPDVE(matROWS,3)];
temp_rightV = reshape(temp_rightV,sum(INPU.vecN,1),[]);

[move_row,~] = find(temp_rightV); % Vector correspond to which index of deflection velocity matrix should be used for each element

v_rot = temp_r(temp_leftV,1).*omega(move_row+1)';
% Translate right edge vertices due to twist
twistXDIST = -temp_r(temp_rightV,1).*cos(del_twist(move_row+1)+vecEDGEPITCH(move_row+1))' + ...
    temp_r(temp_rightV,1).*cos(vecEDGEPITCH(move_row+1))'; % X component of twist
twistZDIST = temp_r(temp_rightV,1).*sin(del_twist(move_row+1)+vecEDGEPITCH(move_row+1))' - ...
    temp_r(temp_rightV,1).*sin(vecEDGEPITCH(move_row+1))'; % Z component of twist 

temp_translate(temp_rightV,1) = twistXDIST';
temp_translate(temp_rightV,3) = twistZDIST';

test(temp_rightV,1) = v_rot.*sin(OUTP.matTWISTGLOB(valTIMESTEP,move_row+1)+vecEDGEPITCH(move_row+1))'.*COND.valDELTIME;
test(temp_rightV,3) = v_rot.*cos(OUTP.matTWISTGLOB(valTIMESTEP,move_row+1)+vecEDGEPITCH(move_row+1))'.*COND.valDELTIME;

% Assign appropriate sign to twist movement
temp_translate(:,1) = xz_sign.*temp_translate(:,1);
temp_translate(:,3) = xz_sign.*temp_translate(:,3);

% ======================================================================= %
% =============== TRYING NEW TRANSLATION FROM TWIST ===================== %
% ======================================================================= %
% temp_translate(:,1) = xz_sign.*test(:,1);
% temp_translate(:,3) = xz_sign.*test(:,3);
% ======================================================================= %
% ======================================================================= %

% Translate right edge vertices due to freestream and bending
translateNTVLST(temp_rightV,1) = COND.valDELTIME.*vecXVEL(move_row+1);
translateNTVLST(temp_rightV,2) = COND.valDELTIME.*vecYVEL(move_row+1);
translateNTVLST(temp_rightV,3) = -1*COND.valDELTIME.*vecZVEL(move_row+1);

%% Move wing and generate new wake elements

% Old trailing edge vertices
MISC.matNEWWAKE(:,:,4) = SURF.matVLST(SURF.matDVE(SURF.vecDVETE>0,4),:);
MISC.matNEWWAKE(:,:,3) = SURF.matVLST(SURF.matDVE(SURF.vecDVETE>0,3),:);

% Old non-planar trailing edge vertices (used to calculate matWADJE)
MISC.matNPNEWWAKE(:,:,4) = SURF.matNPVLST(SURF.matNPDVE(SURF.vecDVETE>0,4),:);
MISC.matNPNEWWAKE(:,:,3) = SURF.matNPVLST(SURF.matNPDVE(SURF.vecDVETE>0,3),:);

% Update SURF.matVLST and SURF.matNTVLST
SURF.matNPVLST = SURF.matNPVLST - (translateNTVLST - temp_translate);

% New non-planar trailing edge vertices (used to calculate matWADJE)
MISC.matNPNEWWAKE(:,:,1) = SURF.matNPVLST(SURF.matNPDVE(SURF.vecDVETE>0,4),:);
MISC.matNPNEWWAKE(:,:,2) = SURF.matNPVLST(SURF.matNPDVE(SURF.vecDVETE>0,3),:);

