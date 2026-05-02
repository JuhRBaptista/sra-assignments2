function [fAttractive, fRepulsive] = VFF(robotPose, targetPose, occMap, windowSize, scale, origin)

    % Gains
    kAtt = 1.0;
    kRep = 0.5;

    % --- Robot position in grid ---
    robotGridX = round(robotPose(1) * scale + origin);
    robotGridY = round(robotPose(2) * scale + origin);

    % --- Attractive force ---
    delta = targetPose - robotPose;
    dist  = norm(delta);

    if dist > 0
        fAttractive = kAtt * (delta / dist);
    else
        fAttractive = [0, 0];
    end

    % --- Repulsive force ---
    fRepulsive = [0, 0];
    halfWindow = floor(windowSize / 2);

    [mapH, mapW] = size(occMap);

    xMin = max(1, robotGridX - halfWindow);
    xMax = min(mapH, robotGridX + halfWindow);
    yMin = max(1, robotGridY - halfWindow);
    yMax = min(mapW, robotGridY + halfWindow);

    for x = xMin:xMax
        for y = yMin:yMax

            occValue = occMap(x, y);
            if occValue == 0
                continue; % skip free cells
            end

            dx = x - robotGridX;
            dy = y - robotGridY;
            d  = norm([dx, dy]);

            if d == 0
                continue; % avoid singularity
            end

            % Repulsive contribution (inverse square law)
            magnitude = kRep * occValue / (d^2);
            direction = [dx, dy] / d;

            fRepulsive = fRepulsive - magnitude * direction;
        end
    end
end