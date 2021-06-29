function [OUTP] = fcnREINIT(OUTP,maxtime,valVEHICLES)
%fcnREINIT Re-initializes parameters when running a case to a new maxtime.
%
% The only-parameters re-initialized are the OUTP variables where
% appropriate. The previous values are kept, and nans fill to the new
% maxtime length
% This has not been tested for multiple vehicles. Also, there may be
% additional parameters, these variable were based on fcnINITIALIZE

%%
% Determine length that must be added on
len = maxtime-size(OUTP.vecCL,1);

% Fill with nans to new length
OUTP.vecCL = [OUTP.vecCL;nan(len,valVEHICLES)];
OUTP.vecCLF = [OUTP.vecCLF;nan(len,valVEHICLES)];
OUTP.vecCLI = [OUTP.vecCLI;nan(len,valVEHICLES)];
OUTP.vecCDI = [OUTP.vecCDI;nan(len,valVEHICLES)];
OUTP.vecE = [OUTP.vecE;nan(len,valVEHICLES)];
OUTP.vecCT = [OUTP.vecCT;nan(len,valVEHICLES)];
OUTP.vecCPI = [OUTP.vecCPI;nan(len,valVEHICLES)];
OUTP.vecCP = [OUTP.vecCP;nan(len,valVEHICLES)];
OUTP.vecCFy = [OUTP.vecCFy;nan(len,valVEHICLES)];
OUTP.vecCFx = [OUTP.vecCFx;nan(len,valVEHICLES)];
OUTP.vecCMy = [OUTP.vecCMy;nan(len,valVEHICLES)];
OUTP.vecCMx = [OUTP.vecCMx;nan(len,valVEHICLES)];
OUTP.vecCTCONV = [OUTP.vecCTCONV;nan(len,valVEHICLES)];

OUTP.vecCLv = [OUTP.vecCLv;nan(len,valVEHICLES)];
OUTP.vecCD = [OUTP.vecCD;nan(len,valVEHICLES)];
OUTP.vecPREQ = [OUTP.vecPREQ;nan(len,valVEHICLES)];
OUTP.vecLD = [OUTP.vecLD;nan(len,valVEHICLES)];
