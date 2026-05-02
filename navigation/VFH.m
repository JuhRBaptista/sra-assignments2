function [steerDirection, binary, alpha] = VFH(currentPose, targetPose, map, searchWindowSize)

    % ── Parameters (tune these) ───────────────────────────────────────────
    alpha       = pi/36;
    numSectors  = round(2*pi/alpha);               % angular resolution (5° per bin)
    smax        = 16;                        % min bins for a "wide" valley
    threshold   = 6;                        % histogram counts below this = free
    smoothSigma = 1.5;                      % gaussian smoothing std dev
    scale       = 20;                       % map scale (pixels/meter)
    origin      = 0;                        % map origin offset

    half  = floor(searchWindowSize / 2);
    b  = 3;
    a  = half*b*sqrt(2);

    % Extract local obstacle points from map window
    xRobot = currentPose(1);
    yRobot = currentPose(2);

    xGrid = round(xRobot * scale) + origin;
    yGrid = round(yRobot * scale) + origin;

    xMin  = max(1, xGrid - half);   
    xMax = min(size(map,1), xGrid + half);
    yMin  = max(1, yGrid - half);   
    yMax = min(size(map,2), yGrid + half);

    window = map(xMin:xMax, yMin:yMax);

    xCenterLocal = xGrid - xMin + 1;
    yCenterLocal = yGrid - yMin + 1;

    [x_map, y_map] = find(window);

    % Build polar histogram 
    h = zeros(1, numSectors);

    for i = 1:length(x_map)
        x = x_map(i);
        y = y_map(i);

        Cxy = window(x, y);


        dx = x - xCenterLocal;
        dy = y - yCenterLocal;

        d  = sqrt(dx^2 + dy^2);

        beta = atan2(dy, dx);                                       % obstacle angle   
        m    = max(0, Cxy^2*(a - b*d));                             % magnitude (closer = stronger)
        k    = mod(round((beta + pi) / alpha), numSectors) + 1;                      % sector index

        h(k) = h(k) + m;
    end

    % Smooth histogram (1D Gaussian, circular)
    h = smoothHistogram(h, smoothSigma);
    binary = h > threshold

    % Project target direction onto histogram
    targetAngle  = atan2(targetPose(2) - yRobot, targetPose(1) - xRobot);
    targetSector = mod(round((targetAngle + pi) / alpha), numSectors) + 1

    % Check if target sector is free
    freeWindow = floor(smax / 2);
    if isSectorRangeFree(h, targetSector, freeWindow, threshold, numSectors)
        steerDirection = targetAngle;
        return;
    end
    
    % Find zero-crossing transitions (valley edges)
    
    edges = findValleyEdges(binary, numSectors);   % sector indices of transitions

    if isempty(edges)
        steerDirection = NaN;   % no free space — stop
        return;
    end

    %Select nearest valley edge to target 
    diffs = min(abs(edges - targetSector), numSectors - abs(edges - targetSector)); 

    [~, idx] = min(diffs);
    kn = edges(idx);

    % Wide or small valley?
    [freeCount, dir] = countFreeBinsFrom(binary, kn, numSectors);

    % Substitui a selecção do kn por isto:
    if freeCount >= smax
        % Wide valley: steer toward centre of free region from kn
        offset       = floor(smax / 2);
        steerSector  = mod(kn + dir*offset - 1, numSectors) + 1;
    else
        % Small valley: find the other edge and steer to centre
        kn2         = mod(kn + dir*freeCount - 1, numSectors) + 1;
        steerSector = round(mod((kn + kn2) / 2 - 1, numSectors)) + 1;
    end
    
    steerDirection = (steerSector - 1) * alpha - pi + alpha / 2;
    
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

function free = isSectorRangeFree(h, centerSector, halfWidth, threshold, numSectors)
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

function [count, dir] = countFreeBinsFrom(binary, startSector, numSectors)
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