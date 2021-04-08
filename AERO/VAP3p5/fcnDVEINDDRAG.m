function [inddrag] = fcnDVEINDDRAG(valTIMESTEP, SURF, WAKE, FLAG)
% Induced dve drag. Function finds induced drag values on each te element. Outputs are not
% non-dimensionalized to q.

%% preliminary stuff
%te elements
idte = (SURF.vecDVETE == 3);

%number of te elements
numte = sum(idte);

%ABC for TE elements only

A = SURF.matCOEFF(idte,1);
B = SURF.matCOEFF(idte,2);
C = SURF.matCOEFF(idte,3);

%% FINDING INDUCED POINTS
% 80% of halfspan of TE elements only
eta8 = SURF.vecDVEHVSPN(idte).*0.8;

%TE vectors of TE elements only
s =( SURF.matVLST(SURF.matDVE(idte,3),:) -SURF.matVLST(SURF.matDVE(idte,4),:) )  ./ repmat((SURF.vecDVEHVSPN(idte).*2),1,3); %?? why is the S vector non-dim. to the span?

% element TE edge midpoint of TE elements only
xte = (SURF.matVLST(SURF.matDVE(idte,3),:) + SURF.matVLST(SURF.matDVE(idte,4),:))/2;

% we need to do a lot of work on the induced points
% New method of moving the TE points of the index (induced DVE) points.
% 17 Oct 2014. Bill B
% This method moves the points in the freestream direction into the plane
% passing through the TE of the inducer having freestream direction
% as the normal
tepoints = zeros(numte,3,3);

% find 3 points along TE of all TE elements
%first layer is left side, second layer is middle, third layer is right
%side
tepoints(:,:,1) = (xte + s.*repmat(-eta8,1,3)); %left side
tepoints(:,:,2) = xte ; %middle
tepoints(:,:,3) = (xte + s.*repmat(eta8,1,3)); %right ride

%% WORKING ON INDUCERS /INDUCED POINTS
% newest_row = [((WAKE.valWNELE-WAKE.valWSIZE)+1):1:WAKE.valWNELE]';
% if on same wing!
% we won't use fcnWDVEVEL because we have to change the induced point for each
% column of wake elements, and change dve type for current timestep to 1. 
% So we will call fcnDVEVEL directly. So we have to set up all points/inducers

%need to repmat te points to get each tepoint numte times (induced)
%this will account for the induction of all the wake elements in the
%current timestep, on all the te points.
% if FLAG.TRI == 1
%     numte = numte*2;
% end

tepoints = repmat(tepoints,[numte,1,1]);

%need to repmat the wing index of te elements (induced)
% if FLAG.TRI == 1
%     tewings = repmat(repmat(SURF.vecDVEWING(idte),[numte,1,1]),2,1,1);
%     
% else
    tewings = repmat(SURF.vecDVEWING(idte),[numte,1,1]);
% end

%dvenum is inducer
%need to keep the inducers index the same as the induced points

newest_row = [((WAKE.valWNELE-WAKE.valWSIZE)+1):1:WAKE.valWNELE]';
dvenum = newest_row(repmat(1:WAKE.valWSIZE,WAKE.valWSIZE,1),:);


%inducers wing number repmat
wwings = WAKE.vecWDVESURFACE(newest_row);

wwings = wwings(repmat(1:WAKE.valWSIZE,WAKE.valWSIZE,1),:);


% now actually moving the point:
% vector from TE of each TE element to each point in tepoints
% order is as follows:
% influence of first dve on (1:numte), then influence of second dve
% on (1:numte), etc.
% to keep this cleaner I move all points, then overwrite the cases when
% the inducers wing is different than the induced points.
% if FLAG.TRI == 1
%     delx  = tepoints-repmat(xte(repmat(1:numte/2,numte/2,1),:),2,1,3);
% else
    delx  = tepoints-repmat(xte(repmat(1:numte,numte,1),:),[1 1 3]);
% end
%project into freestream direction
% temps = dot(delx,repmat(vecUINF,[size(delx,1) 1 3]),2);
% tempb = repmat(temps,1,3,1).* repmat(vecUINF,[size(delx,1) 1 3]); %should this be normalized Uinf?
tempUINF = SURF.matUINF(idte,:)./sqrt(sum(SURF.matUINF(idte,:).^2,2));
temps = dot(delx, repmat(tempUINF(repmat(1:numte,numte,1),:),[1 1 3]), 2);
tempb = repmat(temps,1,3,1).*repmat(tempUINF(repmat(1:numte,numte,1),:),[1 1 3]);

% original te point - tempb should be new te point
newtepoint = tepoints - tempb;

%if inducers wing is different than induced wing, newtepoint = oldtepoint
diffw = (tewings~= wwings);
newtepoint(diffw) = tepoints(diffw);

%we have now accounted for all the current timestep of wake elements, now repmat to
%account for remaining wake rows
%fpg is all points to go into DVEVEL


fpg = repmat(newtepoint,[valTIMESTEP,1,1]);
dvenum = repmat(dvenum,[valTIMESTEP,1,1]);


% Oldest row of wake DVEs are semi-infinite
oldest_row = [1:WAKE.valWSIZE]';

mult = [1:valTIMESTEP]'; %need to renumber old timestep rows
multnew = repmat(mult,[WAKE.valWSIZE*WAKE.valWSIZE,1,1]);
multnew = sort(multnew);
dvenum = dvenum - repmat(WAKE.valWSIZE,size(dvenum,1),1).*(multnew-1);

dvenum = repmat(dvenum,[1 1 3]);%correct inducers index
% take second dimension, move to bottom. then take third dimension and move
% to bottom
fpg = reshape(permute(fpg,[1 3 2]),[],3);
dvenum = reshape(permute(dvenum,[1 3 2]),[],1);

if FLAG.STEADY == 1
    %dve type
    dvetype = ones(length(dvenum),1);

    dvetype(ismember(dvenum, newest_row)) = 1;%FW has this as type 1, but should be 2?

    %setting singfct for post-trailing edge row to 0
    tempwk = WAKE.vecWK(dvenum);
    tempwk(ismember(dvenum, newest_row)) = 0;


    if valTIMESTEP == 1
        dvetype(ismember(dvenum, oldest_row)) = 1;
    else
        dvetype(ismember(dvenum, oldest_row)) = 3;
    end

elseif FLAG.STEADY == 0 || FLAG.STEADY == 2
    %dve type
    dvetype = zeros(length(dvenum),1);

    dvetype(ismember(dvenum, newest_row)) = -2;%FW has this as type 1, but should be 2?

    %setting singfct for post-trailing edge row to 0
    tempwk = WAKE.vecWK(dvenum);
    tempwk(ismember(dvenum, newest_row)) = 0;
    
    if valTIMESTEP == 1
        dvetype(ismember(dvenum, oldest_row)) = 3;
    else
        dvetype(ismember(dvenum, oldest_row)) = -3;
    end    
    
end

%get all velocities %need to set singfct = 0 for le row of elements!!!
[w_ind] = fcnDVEVEL(dvenum, fpg, dvetype, WAKE.matWDVE, WAKE.matWVLST, WAKE.matWCOEFF, tempwk, WAKE.vecWDVEHVSPN, WAKE.vecWDVEHVCRD, WAKE.vecWDVEROLL, WAKE.vecWDVEPITCH, WAKE.vecWDVEYAW, zeros(size(WAKE.vecWDVELESWP)), zeros(size(WAKE.vecWDVETESWP)), SURF.vecDVESYM, FLAG.GPU);

% idxnans = sum(isnan(w_ind),2);
% idxnans = idxnans > 0;

% undo reshape and permute
w_total = permute(reshape(w_ind,[],3,3),[1 3 2]);

%add up influence of every dve on each point (every WSIZE value corresponds
%to the ind. vel on one point
w_wake(:,:,1) = reshape(sum(reshape(w_total(:,:,1)', WAKE.valWSIZE*3, [])',1),3,[])';
w_wake(:,:,2) = reshape(sum(reshape(w_total(:,:,2)', WAKE.valWSIZE*3, [])',1),3,[])';
w_wake(:,:,3) = reshape(sum(reshape(w_total(:,:,3)', WAKE.valWSIZE*3, [])',1),3,[])';

% if FLAG.TRI == 1
%    w_wake = w_wake(1:end/2,:,:) + w_wake((end/2)+1:end,:,:);
% end

%w_wake is [num tedves x 3 x k]

%% INTEGRATION
% //Kutta-Joukowski at 3 edges (left, center, right)
tempa = cross(w_wake,repmat(s,[1 1 3]),2);

gamma(:,1) = A - B.*eta8 + C.*eta8.*eta8;
gamma(:,2) = A;
gamma(:,3) =  A + B.*eta8 + C.*eta8.*eta8;

tempr = tempa.*repmat(permute(gamma,[1 3 2]),[1,3,1]);
% gamma1  = A - B.*eta8 + C.*eta8.*eta8;
% R1 = tempa(:,:,1).*repmat(gamma(:,1),1,3);

% gammao  = A;
% Ro = tempa(:,:,2).*repmat(gamma(:,2),1,3);

% gamma2  = A + B.*eta8 + C.*eta8.*eta8;
% R2 = tempa(:,:,3).*repmat(gamma(:,3),1,3);

% simpsons rule:
R(:,:)  = (tempr(:,:,1)+4.*tempr(:,:,2)+tempr(:,:,3)).*repmat(eta8,[1 3])./3;	
% R(:,:)  = (R1(:,:)+4*Ro(:,:)+R2(:,:)).*repmat(eta8,1,3)./3;	

% plus overhanging parts:
R(:,:) = R(:,:)+((7.*tempr(:,:,1)-8.*tempr(:,:,2)+7.*tempr(:,:,3)).*repmat(SURF.vecDVEHVSPN(idte)-eta8,[1 3])./3);
% R(:,:) = R(:,:)+((7.*R1(:,:)-8.*Ro(:,:)+7.*R2(:,:)).*repmat((SURF.vecDVEHVSPN(idte)-eta8),1,3)./3);

% R(:,1)  = (R1(:,1)+4*Ro(:,1)+R2(:,1)).*eta8./3;			%//Rx
% R(:,2)  = (R1(:,2)+4*Ro(:,2)+R2(:,2)).*eta8./3;			%//Ry
% R(:,3)  = (R1(:,3)+4*Ro(:,3)+R2(:,3)).*eta8./3;			%//Rz

% 		//plus overhanging parts
% R(:,1) = R(:,1)+((7.*R1(:,1)-8.*Ro(:,1)+7.*R2(:,1)).*(SURF.vecDVEHVSPN(idte)-eta8)./3); %//Rx
% R(:,2) = R(:,2)+((7.*R1(:,2)-8.*Ro(:,2)+7.*R2(:,2)).*(SURF.vecDVEHVSPN(idte)-eta8)./3); %//Ry
% R(:,3) = R(:,3)+((7.*R1(:,3)-8.*Ro(:,3)+7.*R2(:,3)).*(SURF.vecDVEHVSPN(idte)-eta8)./3); %//Rz
%% FORCES
% inddrag(:,1) = dot(R,repmat(SURF.matUINF,size(R,1),1),2);
inddrag = zeros(SURF.valNELE,1);
inddrag(idte,1) = dot(R,tempUINF,2);

end %end function
