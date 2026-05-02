function updatePathTrackingPlot(plotHandles, trajectory, robotPose, targetPose, histogram, alpha)

    % Atualizar trajetória
    if ~isempty(trajectory)
        set(plotHandles.trajectory, ...
            'XData', trajectory(:,1), ...
            'YData', trajectory(:,2));
    end

    % Atualizar robô
    set(plotHandles.robot, ...
        'XData', robotPose.x, ...
        'YData', robotPose.y);

    % Atualizar target
    set(plotHandles.target, ...
        'XData', targetPose.x, ...
        'YData', targetPose.y);

    % Atualizar histograma VFH
    if nargin >= 5 && ~isempty(histogram)

        nSectors = length(histogram);

        % Normalizar (evita divisão por zero)
        histNorm = histogram / (max(histogram) + 1e-6) * 0.3;

        % Ângulos dos setores
        angles = (0:nSectors-1) * alpha - pi + alpha/2;

        % Coordenadas polares → cartesianas
        xHist = robotPose.x + histNorm .* cos(angles);
        yHist = robotPose.y + histNorm .* sin(angles);

        % Fechar o polígono
        xHist(end+1) = xHist(1);
        yHist(end+1) = yHist(1);

        set(plotHandles.histogram, ...
            'XData', xHist, ...
            'YData', yHist);
    end

    % Atualizar gráfico
    drawnow limitrate;
end