%Saturation thresholds 

angles_rad = [0.30, 0.52, 0.79];
thresholds = [4, 7, 10];

figure;
plot(angles_rad, thresholds, 'o-', 'LineWidth', 1.5, 'MarkerFaceColor', 'b')
xlabel('Initial angle (rad)')
ylabel('Force threshold (N)')
title('Minimum force required to stabilize vs. initial angle')
grid on

%overlay linear fit to show the ~13 N/rad relationship
hold on
p = polyfit(angles_rad, thresholds, 1);
fitLine = polyval(p, [0 0.9]);
plot([0 0.9], fitLine, '--r')
legend('Measured threshold', 'Linear fit')
fprintf('Fit slope: %.2f N/rad, intercept: %.2f N\n', p(1), p(2))