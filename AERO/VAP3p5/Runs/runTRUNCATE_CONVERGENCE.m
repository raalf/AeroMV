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

maxtime = [80 100 120 140 160 180 200 240 280 340 400];
dif_trunc = [-60 -40 -20 0];

for i = 1:length(dif_trunc) % There must be a better way to preallocate
    for k = 1:2
        for j = 1:length(maxtime)
            DATA(j,i,k).CT= [];
            DATA(j,i,k).CP = [];
            DATA(j,i,k).CQ = [];
            DATA(j,i,k).CMx = [];
            DATA(j,i,k).CMy = [];
            DATA(j,i,k).CNx = [];
            DATA(j,i,k).CNy = [];
            DATA(j,i,k).OUTP = [];
        end
    end
end

for i = 1:length(dif_trunc)
    temp_dif_trunce = dif_trunc(i);
    for k = 1:2
        flag_relax = k-1;
        parfor j = 1:length(maxtime)
            VAP_IN = [];
            VAP_IN.RELAX = flag_relax;
            
            VAP_IN.valMAXTIME = maxtime(j);
            VAP_IN.valSTARTFORCES = VAP_IN.valMAXTIME-20;
            
            if temp_dif_trunce == 0
                VAP_IN.TRUNCATE = 0;
            else
                VAP_IN.TRUNCATE = 1;
                VAP_IN.valTIMETRUNC = maxtime(j)-temp_dif_trunce;
            end
            
            
            VAP_IN.vecROTORRPM = rpm;
            VAP_IN.vecVEHALPHA = -AOA;
            VAP_IN.vecVEHVINF = vel;
            VAP_IN.valDELTIME = 1/((rpm/60)*20); %20 timesteps per rev
            
            
            OUTP = fcnVAP_MAIN(filename, VAP_IN);
            
            DATA(j,i,k).CT= OUTP.vecCT_AVG;
            DATA(j,i,k).CP = OUTP.vecCP_AVG;
            DATA(j,i,k).CQ = OUTP.vecCP_AVG/(2*pi*rpm/60);
            DATA(j,i,k).CMx = OUTP.vecCMx_AVG;
            DATA(j,i,k).CMy = OUTP.vecCMy_AVG;
            DATA(j,i,k).CNx = OUTP.vecCFx_AVG;
            DATA(j,i,k).CNy = OUTP.vecCFy_AVG;
            DATA(j,i,k).OUTP = OUTP;
            
        end
        fprintf('Done Parfor iteration: Relax = %d, Dif Trunc %d \n',k,dif_trunc(i))
    end
end