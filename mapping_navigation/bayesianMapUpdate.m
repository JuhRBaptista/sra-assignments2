function [probMap] = bayesianMapUpdate(tbot, map, params)

    l_occ   =  0.65;   % log-odds para célula ocupada
    l_free  = -0.65;   % log-odds para célula livre
    l_prior =  0.0;    % prior (p = 0.5)
    l_min   = -5.0;    % clamp para evitar instabilidade numérica
    l_max   =  5.0;

    % O mapa agora guarda log-odds, não probabilidades
    % Probabilidade -> Log-odds
    logOddsMap = log(map ./ (1 - map));

    [xRobot, yRobot, theta] = tbot.readPose();
    theta = normalizeAngle(theta);

    [~, lddata] = tbot.readLidar();
    vidx        = tbot.getInRangeLidarDataIdx(lddata);
    scans       = lddata.Cartesian(vidx, :);
    xScans      = scans(:, 1);
    yScans      = scans(:, 2);

    % Robot frame -> world coordinates
    xWorld = xRobot + xScans*cos(theta) - yScans*sin(theta);
    yWorld = yRobot + xScans*sin(theta) + yScans*cos(theta);

    % World -> grid coordinates
    xScaled = round(xWorld * params.scale) + params.origin;
    yScaled = round(yWorld * params.scale) + params.origin;

    % Robot position in grid
    xRobotGrid = round(xRobot * params.scale) + params.origin;
    yRobotGrid = round(yRobot * params.scale) + params.origin;

    % --- Bayesian update para cada raio LIDAR ---
    for i = 1:length(xScaled)

        x_occ = xScaled(i);
        y_occ = yScaled(i);

        % 1) Células LIVRES ao longo do raio (Bresenham)
        free_cells = bresenham(xRobotGrid, yRobotGrid, x_occ, y_occ);

        % Filtrar células dentro dos limites do mapa
        if ~isempty(free_cells)
            valid_free = free_cells(:,1) > 0 & free_cells(:,1) <= params.mapSize & ...
                         free_cells(:,2) > 0 & free_cells(:,2) <= params.mapSize;
            free_cells = free_cells(valid_free, :);

            % Update log-odds: l = l + l_free - l_prior
            idx_free = sub2ind(size(logOddsMap), free_cells(:,1), free_cells(:,2));
            logOddsMap(idx_free) = logOddsMap(idx_free) + l_free - l_prior;
        end

        % 2) Célula OCUPADA no endpoint
        if x_occ > 0 && x_occ <= params.mapSize && ...
           y_occ > 0 && y_occ <= params.mapSize

            idx_occ = sub2ind(size(logOddsMap), x_occ, y_occ);
            logOddsMap(idx_occ) = logOddsMap(idx_occ) + l_occ - l_prior;
        end
    end

    % Clamp para evitar instabilidade numérica
    logOddsMap = max(l_min, min(l_max, logOddsMap));

    % Converter log-odds -> probabilidade para display e para o VFF/VFH
    probMap = 1 ./ (1 + exp(-logOddsMap));

end
