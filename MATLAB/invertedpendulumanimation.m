figure; 
axis equal;
axis([-2 2 -1 1.5])
hold on 
grid on

%create objects before loop, then repeatedly update position
%create cart object
cartwidth = 0.3; 
cartheight = 0.15;
carthandle = rectangle('Position', [0-cartwidth/2, 0, cartwidth,... 
  cartheight],'FaceColor', [0.2 0.4 0.8]);

%create rod object
rodhandle = line([0 0], [0 0,], 'LineWidth',2, 'Color', 'k');
bobhandle = plot(0, 0, "o", 'MarkerSize', 12, 'MarkerFaceColor', 'r');   

%idx = find(abs(y(:,1)) > 50, 1);   % find first point where x exceeds a plottable range
%y_trimmed = y(1:idx, :);
%t_trimmed = t(1:idx);

%create animation loop
for k = 1:3:length(t)  %(start:steps: time)
    xcart = y_ideal(k,1); %xcart and theta at time k
    theta = y_ideal(k,3);
    
    %y_ideal uses y from original code
    %y uses y from simulink (saturated)


    %update cart position
    set(carthandle, 'Position', [xcart-cartwidth/2, 0, ...
        cartwidth, cartheight]); %animation - set() takes an existing 
                                 %handle and changes its position
    %set pendulum positions                      
    pivotx = xcart;
    pivoty = cartheight;                    
    tipx = pivotx + L*sin(theta);
    tipy = pivoty + L*cos(theta);
    
    fprintf('k=%d, xcart=%.3f, theta=%.3f, tipx=%.3f\n', ...
        k, xcart, theta, tipx)

    %update pendulum position
    set(rodhandle, 'XData', [pivotx tipx], 'YData', [pivoty tipy]);
    set(bobhandle, 'XData', tipx, 'YData', tipy);

    drawnow
    pause(0.1)
end

