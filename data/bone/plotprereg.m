clear;

load('amode_areasphere_554cm');

figure1 = figure(1);
figure1.WindowState  = 'maximized';

for i=1:length(preregistrationArea)
    subplot(1,3, i);
    plot3( preregistrationArea{i}(:,1), ...
           preregistrationArea{i}(:,2), ...
           preregistrationArea{i}(:,3), ...
           '.r', ...
           'Tag', 'plot_prereg');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    grid on; axis equal;
end