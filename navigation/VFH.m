function [steerAngle, binaryHist, sectorWidth] = VFH(robotPose, targetPose, occMap, windowSize)

    % Parameters 
    sectorWidth    = pi / 36;                 % angular resolution (~5 deg)
    numSectors     = round(2*pi / sectorWidth);
    valleyMinWidth = 16;
    threshold      = 0.6;
    smoothSigma    = 1.5;
    scale          = 20;
    origin         = 0;

    % Window extraction
    halfWindow = floor(windowSize / 2);

    robotGridX = round(robotPose(1) * scale) + origin;
    robotGridY = round(robotPose(2) * scale) + origin;

    xMin = max(1, robotGridX - halfWindow);
    xMax = min(size(occMap,1), robotGridX + halfWindow);
    yMin = max(1, robotGridY - halfWindow);
    yMax = min(size(occMap,2), robotGridY + halfWindow);

    localMap = occMap(xMin:xMax, yMin:yMax);

    centerX = robotGridX - xMin + 1;
    centerY = robotGridY - yMin + 1;

    [obsX, obsY] = find(localMap);

    % Polar histogram
    histogram = zeros(1, numSectors);

    b = 3;
    a = halfWindow * b * sqrt(2);
    

    for i = 1:length(obsX)

        occValue = localMap(obsX(i), obsY(i));

        dx = obsX(i) - centerX;
        dy = obsY(i) - centerY;
        d  = norm([dx, dy]);

        if d == 0
            continue;
        end

        angle = atan2(dy, dx);

        % Obstacle influence (closer = stronger)
        magnitude = max(0, occValue^2 * (a - b*d));

        sector = mod(round((angle + pi) / sectorWidth), numSectors) + 1;

        histogram(sector) = histogram(sector) + magnitude;
    end

    % Smooth histogram 
    histogram = smoothHistogram(histogram, smoothSigma);

    % Binary occupancy
    binaryHist = histogram > threshold;

    % Target direction
    targetAngle = atan2(targetPose(2) - robotPose(2), ...
                        targetPose(1) - robotPose(1));

    targetSector = mod(round((targetAngle + pi) / sectorWidth), numSectors) + 1;

    % If target direction is free → go straight
    if isSectorFree(histogram, targetSector, floor(valleyMinWidth/2), threshold, numSectors)
        steerAngle = targetAngle;
        return;
    end

    % Find valleys 
    valleyEdges = findValleyEdges(binaryHist, numSectors);

    if isempty(valleyEdges)
        steerAngle = NaN; % no path available
        return;
    end

    % Select closest valley to target
    diff = min(abs(valleyEdges - targetSector), ...
               numSectors - abs(valleyEdges - targetSector));

    [~, idx] = min(diff);
    startSector = valleyEdges(idx);

    % Count valley width
    [freeCount, direction] = countFreeBins(binaryHist, startSector, numSectors);

    if freeCount >= valleyMinWidth
        % Wide valley → go inside it
        offset      = floor(valleyMinWidth / 2);
        steerSector = mod(startSector + direction*offset - 1, numSectors) + 1;
    else
        % Narrow valley → go to its center
        endSector   = mod(startSector + direction*freeCount - 1, numSectors) + 1;
        steerSector = round(mod((startSector + endSector)/2 - 1, numSectors)) + 1;
    end

    % Convert sector to angle
    steerAngle = (steerSector - 1) * sectorWidth - pi + sectorWidth/2;
end

function hs = smoothHistogram(h, sigma)
    % Circular 1D Gaussian smoothing
    halfW  = ceil(3 * sigma);
    kernel = exp(-(-halfW:halfW).^2 / (2 * sigma^2));
    kernel = kernel / sum(kernel);
    n      = length(h);
    hs     = zeros(1, n);
    for i = 1:n
        for j = 1:length(kernel)
            idx    = mod(i + j - halfW - 2, n) + 1;
            hs(i)  = hs(i) + kernel(j) * h(idx);
        end
    end
end

function free = isSectorFree(h, centerSector, halfWidth, threshold, numSectors)
    free = true;
    for offset = -halfWidth:halfWidth
        k = mod(centerSector + offset - 1, numSectors) + 1;
        if h(k) > threshold
            free = false;
            return;
        end
    end
end

function edges = findValleyEdges(binary, numSectors)
    edges = [];
    for k = 1:numSectors
        curr = binary(k);
        prev = binary(mod(k - 2, numSectors) + 1);
        if prev == 1 && curr == 0      % falling edge = início do vale
            edges = [edges, k];
        elseif prev == 0 && curr == 1  % rising edge = fim do vale
            last_free = mod(k - 2, numSectors) + 1;
            edges = [edges, last_free];
        end
    end
end

function [count, dir] = countFreeBins(binary, startSector, numSectors)
    count = 0;
    k = startSector;
    next = mod(k, numSectors) + 1;
    if binary(next) == 0
        dir = 1;
    else
        dir = -1;
    end

    while binary(k) == 0
        count = count + 1;
        k = mod(k - 1 + dir, numSectors) + 1;
        if k == startSector; break; end   % full circle
    end
end