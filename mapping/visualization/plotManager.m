function ui = plotManager(map, params)

    ui.fig = figure('KeyPressFcn', @keyboardControl);

    ui.img = imagesc(map');
    colormap(flipud(gray));
    axis equal;
    set(gca, 'YDir', 'normal');
    hold on;

    ui.robot = plot(0,0,'ro','MarkerSize',8,'LineWidth',2);
    ui.arrow = quiver(0,0,0,0,'r','LineWidth',2);

end

function update(ui, map, robotGrid, theta)

    arrowLen = 10;

    % 1) MAPA
    set(ui.img, 'CData', map');

    % 2) ROBÔ
    set(ui.robot, ...
        'XData', robotGrid(1), ...
        'YData', robotGrid(2));

    % 3) SETA ORIENTAÇÃO
    set(ui.arrow, ...
        'XData', robotGrid(1), ...
        'YData', robotGrid(2), ...
        'UData', arrowLen * cos(theta), ...
        'VData', arrowLen * sin(theta));

    % 4) redraw eficiente
    drawnow limitrate;

end