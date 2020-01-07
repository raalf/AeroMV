% Testing rotation matrix

% Sample vector in body frame:
vec = [1 3 5]';

% Pitch +ve90deg
R1 = fcnEUL2R([0 90 0],3,1);
vec1 = R1*vec;

% Pitch -90deg
R2 = fcnEUL2R([0 -90 0],3,1);
vec2 = R2*vec;

% Roll 90deg
R3 = fcnEUL2R([90 0 0],3,1);
vec3 = R3*vec;

% Roll -ve90deg
R4 = fcnEUL2R([-90 0 0],3,1);
vec4 = R4*vec;

figure(1)
clf(1)
hold on
quiver3(0,0,0,vec(1),vec(2),vec(3))
quiver3(0,0,0,vec1(1),vec1(2),vec1(3))
quiver3(0,0,0,vec2(1),vec2(2),vec2(3))
quiver3(0,0,0,vec3(1),vec3(2),vec3(3))
quiver3(0,0,0,vec4(1),vec4(2),vec4(3))
legend('Original','Pitch +90deg', 'Pitch -90deg','Roll +90deg', 'Roll -90deg')
grid on 
box on
ylabel('y-dir')
xlabel('x-dir')
zlabel('z-dir')
axis equal
hold off