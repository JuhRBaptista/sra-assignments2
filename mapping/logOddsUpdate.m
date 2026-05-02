function logOdds = logOddsUpdate(logOdds, robotGrid, occGrid, params)

    l_occ  = 0.65;
    l_free = -0.65;
    l_min  = -5.0;
    l_max  = 5.0;

    x0 = robotGrid(1);
    y0 = robotGrid(2);

    for i = 1:size(occGrid,2)

        x1 = occGrid(1,i);
        y1 = occGrid(2,i);

        % Ray tracing (free space) 
        freeCells = bresenham(x0, y0, x1, y1);

        % Valid bounds
        validMask = freeCells(:,1) > 0 & freeCells(:,1) <= params.mapSize & ...
                    freeCells(:,2) > 0 & freeCells(:,2) <= params.mapSize;

        freeCells = freeCells(validMask, :);   % keep Nx2

        if ~isempty(freeCells)
            idxFree = sub2ind(size(logOdds), ...
                              freeCells(:,1), freeCells(:,2));

            logOdds(idxFree) = logOdds(idxFree) + l_free;
        end

        % Occupied cell
        if x1 > 0 && x1 <= params.mapSize && ...
           y1 > 0 && y1 <= params.mapSize

            logOdds(x1,y1) = logOdds(x1,y1) + l_occ;
        end
    end

    % Clamp values
    % logOdds = clamp(logOdds, l_min, l_max);
end


function x = clamp(x, mn, mx)
    x = max(mn, min(mx, x));
end