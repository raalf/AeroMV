

% Input filename
filename = 'DJI_Matrice_210_RTK';
% Dataset
load('DATA/Matrice_210_RTK_Dataset/July3_2020_Flight_1.mat','Flight_Data','density','flight_segments')

Euler = [];
BODY_RATES2 = [];
BODY_RATES = [];
for flight_num = 1:6
    
    Euler = [Euler;Flight_Data(1,flight_num).Euler_Angles];
    BODY_RATES2 = [BODY_RATES2;Flight_Data(1,flight_num).Body_Rates];
    BODY_RATES = [BODY_RATES; diff(Euler)*50];
end

figure(1)
clf(1)
hold on
histogram([BODY_RATES2(:,1);BODY_RATES2(:,2)]);
xlabel('Pitch & Roll Rates (rad/s)')
title('DJI Measured Pitch and Roll Rates')
ylabel('Number of Samples')
xlim([-1 1])
grid on
box on
hold off


figure(2)
clf(2)
hold on
histogram([BODY_RATES(:,1);BODY_RATES(:,2)]);
title('DJI Calculated Pitch and Roll Rates')
ylabel('Number of Samples')
xlabel('Pitch & Roll Rates (rad/s)')
xlim([-0.5 0.5])
grid on
box on
hold off
