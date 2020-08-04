function [] = fcnRUN_DIR()
%fcnRUN_DIR changes makes it so that you can use the RUN functions within
%the RUN folder rather than the main folder of AeroMV

dir = pwd;

% If running from the RUN folder, switch to the main folder (which will
% later add the RUN folder to the path)
if strcmp(dir(end-3:end),'\RUN')
    cd(dir(1:end-4))
end

end

