% Get zero lift line
clear,clc
load('Matrice_Airfoil')

for i = 1:length(Re_range)
    idx = ~isnan(pol(:,1,i));
    [CL, sortIDX] = sort(pol(idx,2,i));
    alpha = pol(idx,1,i);
    alpha = alpha(sortIDX);
    if i < 3
        idx_alpha = (alpha<7 & alpha >-8);
    else
        idx_alpha = (alpha<7 & alpha >-10);
    end
    alpha_0(i) = interp1(CL(idx_alpha),alpha(idx_alpha),0);
end

figure(1)
clf(1)
hold on
plot(Re_range,alpha_0,'*k')
xlabel('Re_range')
ylabel('alpha_0')
grid on
box on
hold off
 

rpm = 3000:500:6000;
for i = 1:length(rpm)
diam = 0.4572;
mu = 1.46e-05;
r_dist = linspace(0.024063,diam/2,18);

chord =[0.0356968918856080,0.0427106226458963,0.0472358502673797,0.0460027602883050,0.0407887030923041,0.0362698535224366,0.0328820451057894,0.0300544422227389,0.0276818070216229,0.0257333122529644,0.0240059232736573,0.0222859753545687,0.0210263101604278,0.0196242018135317,0.0184113089979075,0.0172026682166938,0.0163044259474541,0.0125730000000000];
V = (rpm(i)/60).*2*pi.*r_dist;
Re = V.*chord./mu;


alpha_0_dist = interp1(Re_range,alpha_0,Re);

figure(2)
hold on
plot(r_dist/(diam/2),alpha_0_dist,'linewidth',2)
hold off
end
figure(2)
hold on
leg = legend(num2str(rpm'),'location','northwest');
grid on
grid minor
box on
ylabel('Zero lift angle of attack')
xlabel('r/R')
axis tight
title(leg,'RPM')
hold off
