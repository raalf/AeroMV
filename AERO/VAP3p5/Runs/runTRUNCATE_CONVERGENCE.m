clear,clc
%% Sort out folder and paths
folder = pwd;
if strcmp(folder(end-4:end),'\Runs')
    addpath(folder)
    cd(folder(1:end-4))
end

filename = 'inputs/Matrice_210_RTK_Rotor.vap';

AOA = 10;
rpm = 4000;
vel = 5;

vecmaxtime = fliplr([80 100 120 140 160 180 200 240 280 340 400]);
vecdif_trunc = [-60 -40 -20 0];
vecrelax = [0 1];

% vecmaxtime = [20 21];
% vecdif_trunc = [-19 -18];
% vecrelax = [0 1];

[x,y,z] = meshgrid(vecmaxtime,vecdif_trunc,vecrelax);
maxtime = reshape(x,numel(x),1,1);
dif_trunc = reshape(y,numel(y),1,1);
relax = reshape(z,numel(z),1,1);

for i = 1:length(maxtime) % There must be a better way to preallocate
            DATA(i).CT= [];
            DATA(i).CP = [];
            DATA(i).CQ = [];
            DATA(i).CMx = [];
            DATA(i).CMy = [];
            DATA(i).CNx = [];
            DATA(i).CNy = [];
            DATA(i).OUTP = [];
            DATA(i).MAXTIME = [];
            DATA(i).RELAX = [];
            DATA(i).TRUNCATE = [];
end


parfor i = 1:length(maxtime)
            
            VAP_IN = [];
            VAP_IN.RELAX = relax(i);
            
            VAP_IN.valMAXTIME = maxtime(i);
            VAP_IN.valSTARTFORCES = VAP_IN.valMAXTIME-20;
            
            if dif_trunc(i) == 0
                VAP_IN.TRUNCATE = 0;
            else
                VAP_IN.TRUNCATE = 1;
                VAP_IN.valTIMETRUNC = maxtime(i)+dif_trunc(i);
            end
            
            
            VAP_IN.vecROTORRPM = rpm;
            VAP_IN.vecVEHALPHA = -AOA;
            VAP_IN.vecVEHVINF = vel;
            VAP_IN.valDELTIME = 1/((rpm/60)*20); %20 timesteps per rev
            
            
            OUTP = fcnVAP_MAIN(filename, VAP_IN);
            
            DATA(i).CT= OUTP.vecCT_AVG;
            DATA(i).CP = OUTP.vecCP_AVG;
            DATA(i).CQ = OUTP.vecCP_AVG/(2*pi*rpm/60);
            DATA(i).CMx = OUTP.vecCMx_AVG;
            DATA(i).CMy = OUTP.vecCMy_AVG;
            DATA(i).CNx = OUTP.vecCFx_AVG;
            DATA(i).CNy = OUTP.vecCFy_AVG;
            DATA(i).OUTP = OUTP;
            DATA(i).MAXTIME = maxtime(i);
            DATA(i).RELAX = relax(i);
            DATA(i).TRUNCATE = dif_trunc(i);

            fcnSAVE(DATA(i),strcat('SaveFiles',num2str(relax(i)),'_Truncate',num2str(dif_trunc(i)),'Maxtime',num2str(maxtime(i))))
            fprintf('Done Case: Maxtime: %d, Relax = %d, Dif Trunc = %d \n',maxtime(i),relax(i),dif_trunc(i))
            
    
end
save('TruncateConvergenceStudy')