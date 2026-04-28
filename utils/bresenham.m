function cells = bresenham(x0, y0, x1, y1)
    if abs(x1 - x0) >= abs(y1 - y0)
        cells = horizontalLine(x0, y0, x1, y1);
    else
        cells = verticalLine(x0, y0, x1, y1);
    end
end

function cells = horizontalLine(x0, y0, x1, y1)
    cells = [];

    % Always sweep left to right
    if x0 > x1
        [x0, x1] = deal(x1, x0);
        [y0, y1] = deal(y1, y0);
    end

    dx = x1 - x0;
    dy = y1 - y0;

    if dy < 0
        dir = -1;
    else
        dir = 1;
    end
    dy = abs(dy);

    y = y0;
    p = 2*dy - dx;

    % i goes from 0 to dx-1: starts at (x0,y0), stops BEFORE endpoint
    for i = 0:dx-1
        cells = [cells; [x0 + i, y]];
        if p >= 0
            y = y + dir;
            p = p - 2*dx;
        end
        p = p + 2*dy;
    end
end

function cells = verticalLine(x0, y0, x1, y1)
    cells = [];

    % Always sweep bottom to top
    if y0 > y1
        [x0, x1] = deal(x1, x0);
        [y0, y1] = deal(y1, y0);
    end

    dx = x1 - x0;
    dy = y1 - y0;

    if dx < 0
        dir = -1;
    else
        dir = 1;
    end
    dx = abs(dx);

    x = x0;
    p = 2*dx - dy;

    % i goes from 0 to dy-1: starts at (x0,y0), stops BEFORE endpoint
    for i = 0:dy-1
        cells = [cells; [x, y0 + i]];
        if p >= 0
            x = x + dir;
            p = p - 2*dy;
        end
        p = p + 2*dx;
    end
end