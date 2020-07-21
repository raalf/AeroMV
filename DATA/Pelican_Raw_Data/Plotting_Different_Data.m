load('smoothed_flight.mat')
smoothed = pelican_flights;
load('trimmed_flights.mat')
trimmed = pelican_flights;
load('C:\Users\Devin\Documents\AeroMV\DATA\Pelican_Dataset\AscTec_Pelican_Flight_Dataset.mat')
onlinedata = flights;

figure(1)
clf(1)
i = 23;
for j = 1:3
    subplot(3, 1, j);
    hold off;
    plot(diff(trimmed{i}.Pos(:,j))./(1/100),'.-');
    hold on;
    plot(diff(smoothed{i}.Pos(:,j))./(1/100),'.-');
    plot(onlinedata{i}.Vel(:,j),'.-');
    if j == 1
        ylabel('Velocity (x-dir)')
    elseif j == 2
        ylabel('Velocity (y-dir)')
    else
        ylabel('Velocity (z-dir)')
    end
    grid on;
end 
legend('Trimmed','Smoothed','Onlinedata')

figure(2)
clf(2)
i = 23;
for j = 1:3
    subplot(3, 1, j);
    hold off;
    plot(diff(trimmed{i}.Euler(:,j))./(1/100),'.-');
    hold on;
    plot(diff(smoothed{i}.Euler(:,j))./(1/100),'.-');
    plot(onlinedata{i}.Euler_Rates(:,j),'.-');
    if j == 1
        ylabel('Roll Rate')
    elseif j == 2
        ylabel('Pitch Rate')
    else
        ylabel('Yaw Rate')
    end
    grid on;
end
legend('Trimmed','Smoothed','Onlinedata')