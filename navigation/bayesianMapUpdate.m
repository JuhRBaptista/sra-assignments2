function logOddsMap = bayesianMapUpdate(xRobotGrid, yRobotGrid, xScaled, yScaled, logOddsMap, params)

    % PARAMETERS
    l_occ   =  0.65;
    l_free  = -0.65;
    l_min   = -5.0;
    l_max   =  5.0;
    l_init = 0.5;

    % MAP UPDATE 
    for i = 1:length(xScaled)

        x_occ = xScaled(i);
        y_occ = yScaled(i);

        % Skip out-of-bounds
        if x_occ <= 0 || x_occ > params.mapSize || ...
           y_occ <= 0 || y_occ > params.mapSize
            continue;
        end

        % FREE CELLS (ray tracing) 
        free_cells = bresenham(xRobotGrid, yRobotGrid, x_occ, y_occ);

        if size(free_cells,1) > 1

            valid = free_cells(:,1) > 0 & free_cells(:,1) <= params.mapSize & ...
                    free_cells(:,2) > 0 & free_cells(:,2) <= params.mapSize;

            free_cells = free_cells(valid,:);

            if ~isempty(free_cells)
                idx = sub2ind(size(logOddsMap), free_cells(:,1), free_cells(:,2));
                logOddsMap(idx) = logOddsMap(idx) + l_free + l_init;
            end
        end

        logOddsMap(x_occ, y_occ) = logOddsMap(x_occ, y_occ) + l_occ + l_init;

    end

    % Clamp 
    logOddsMap = max(l_min, min(l_max, logOddsMap));

end

