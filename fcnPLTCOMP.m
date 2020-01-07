function  fcnPLTCOMP(INFO,fignum)
% This function plots the components as spheres or cylinders

% Plot sphere
figure(fignum)
hold on
if strcmpi(INFO.strTYPE,'Sphere')
    [x,y,z] = sphere;
    x = x*INFO.valDIAM;
    y = y*INFO.valDIAM;
    z = z*INFO.valDIAM;
    surf(x+INFO.vecLOCATION(1),y+INFO.vecLOCATION(2),z+INFO.vecLOCATION(3))

% Plot cylinders
elseif strcmpi(INFO.strTYPE,'Cylinder')
	LEN = sqrt((INFO.matBEGIN(:,1)-INFO.matEND(:,1)).^2+(INFO.matBEGIN(:,2)-INFO.matEND(:,2)).^2+(INFO.matBEGIN(:,3)-INFO.matEND(:,3)).^2);
    for i = 1:size(INFO.matBEGIN,1)
        [X,Y,Z] = cylinder2P(INFO.valDIAM/2, 50,INFO.matBEGIN(i,:),INFO.matEND(i,:));
        surf(X,Y,Z)
    end
end
hold off

end

