function logOdds = logOddsUpdate(logOdds, robotGrid, occGrid, params)

    l_occ  = 0.65;
    l_free = -0.65;
    l_min = -5.0;
    l_max = 5.0;

    x0 = robotGrid(1);
    y0 = robotGrid(2);

    for i = 1:size(occGrid,2)

        x1 = occGrid(1,i);
        y1 = occGrid(2,i);

        if ~inBounds(x1,y1,params), continue; end

        freeCells = bresenham(x0, y0, x1, y1);

        if ~isempty(freeCells)

            idxFree = sub2ind(size(logOdds), ...
                freeCells(:,1), freeCells(:,2));

            logOdds(idxFree) = logOdds(idxFree) + l_free;

        end

        logOdds(x1,y1) = logOdds(x1,y1) + l_occ;

    end

    logOdds = clamp(logOdds, l_min, l_max);

end

function ok = inBounds(x,y,params)
    ok = x>0 && x<=params.mapSize && y>0 && y<=params.mapSize;
end

function x = clamp(x, mn, mx)
    x = max(mn, min(mx, x));
end