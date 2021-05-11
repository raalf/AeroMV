function PERF = fcnRUNVAPSINGLE(GEOM,AIR,PERF,STATE)
%fcnRUNVAP sets up and runs VAP3p5 for each rotor individually

timestepsperrev = 20; % Number of timesteps per revolution
maxtime = 40;
FOLDER_ADDRESS = pwd;
cd(strcat(FOLDER_ADDRESS,'\AERO\VAP3p5'))
for i = 1:length(STATE.RPM)
    VAP_IN = [];
    VAP_IN.RELAX = false;
    VAP_IN.valMAXTIME = maxtime;
    VAP_IN.valSTARTFORCES = floor(maxtime-timestepsperrev);
    
    
    VAP_IN.valDENSITY = AIR.density; % Air density [kg/m^3]
    VAP_IN.valKINV = AIR.kinvisc;
    VAP_IN.vecVEHVINF = STATE.VEL_ROTOR_MAG(i);
    VAP_IN.vecROTORRPM = STATE.RPM(i);
    VAP_IN.vecVEHALPHA = -1*STATE.AOA_R(i);
    VAP_IN.valDELTIME = 1/((STATE.RPM(i)/60)*timestepsperrev);
    
    OUTP = fcnVAP_MAIN(strcat('inputs\',GEOM.ROTOR.strNAME,'.vap'), VAP_IN);
    PERF.ROTOR(i).T = OUTP.vecCT_AVG*((STATE.RPM(i)/60).^2).*((GEOM.ROTOR.vecDIAM(i)).^4)*AIR.density;
    PERF.ROTOR(i).Nx = OUTP.vecCFx_AVG*((STATE.RPM(i)/60).^2).*((GEOM.ROTOR.vecDIAM(i)).^4)*AIR.density;
    PERF.ROTOR(i).Ny = OUTP.vecCFy_AVG*((STATE.RPM(i)/60).^2).*((GEOM.ROTOR.vecDIAM(i)).^4)*AIR.density;
    PERF.ROTOR(i).Q =  (OUTP.vecCP_AVG/(2*pi*STATE.RPM(i)/60)).*((STATE.RPM(i)/60).^2).*((GEOM.ROTOR.vecDIAM(i)).^5)*AIR.density;
    PERF.ROTOR(i).P = OUTP.vecCP_AVG.*((STATE.RPM(i)/60).^3).*((GEOM.ROTOR.vecDIAM(i)).^5)*AIR.density;
    PERF.ROTOR(i).Mx = OUTP.vecCMx_AVG.*((STATE.RPM(i)/60).^2).*((GEOM.ROTOR.vecDIAM(i)).^5)*AIR.density;
    PERF.ROTOR(i).My = OUTP.vecCMy_AVG.*((STATE.RPM(i)/60).^2).*((GEOM.ROTOR.vecDIAM(i)).^5)*AIR.density;
end


cd(FOLDER_ADDRESS)
