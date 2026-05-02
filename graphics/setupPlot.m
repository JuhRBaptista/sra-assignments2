% function [handles] = setupPlot(map, pathWorld, origin, scale)
%     figure; hold on;
% 
%     % Mapa em coordenadas mundo
%     [rows, cols] = size(map);
%     x_extent = [origin/scale, (cols-1+origin)/scale];
%     y_extent = [origin/scale, (rows-1+origin)/scale];
%     imagesc(x_extent, y_extent, map');
%     colormap(flipud(gray));
%     set(gca, 'YDir', 'normal');
% 
%     % Caminho desejado
%     handles.path   = plot(pathWorld(:,1), pathWorld(:,2), 'r--', 'LineWidth', 2);
% 
%     % Trajetória do robô (começa vazia)
%     handles.traj   = plot(NaN, NaN, 'b',  'LineWidth', 2);
% 
%     % Posição atual
%     handles.robot  = plot(NaN, NaN, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
% 
%     % Target intermediário
%     handles.target = plot(NaN, NaN, 'go', 'MarkerSize', 10, 'LineWidth', 2);
% 
%     axis equal; grid on;
%     legend('Desired Path', 'Robot Trajectory', 'Robot Position', 'Target');
%     xlabel('x (m)'); ylabel('y (m)');
%     title('Real-time trajectory tracking');
% end

function handles = setupPlot(map, pathWorld, origin, scale)
    figure; hold on;

    [rows, cols] = size(map);
    x_extent = [origin/scale, (cols-1+origin)/scale];
    y_extent = [origin/scale, (rows-1+origin)/scale];
    imagesc(x_extent, y_extent, map');
    colormap(flipud(gray));
    set(gca, 'YDir', 'normal');

    handles.path   = plot(pathWorld(:,1), pathWorld(:,2), 'r--', 'LineWidth', 2);
    handles.traj   = plot(NaN, NaN, 'b',  'LineWidth', 2);
    handles.robot  = plot(NaN, NaN, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    handles.target = plot(NaN, NaN, 'go', 'MarkerSize', 10, 'LineWidth', 2);
    % inside setupPlot, after imagesc(...)
    handles.mapImg = imagesc(x_extent, y_extent, map');

    % Histograma — começa vazio, será preenchido em updatePlot
    handles.hist = plot(NaN, NaN, 'm-', 'LineWidth', 1.5);

    axis equal; grid on;
    legend('Desired Path', 'Robot Trajectory', 'Robot Position', 'Target', 'VFH Histogram');
    xlabel('x (m)'); ylabel('y (m)');
    title('Real-time trajectory tracking');
end