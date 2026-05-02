function mapBuildingWithTracking(tbot, params, path, savePath, avoidance)

    if nargin < 5, avoidance = "vfh"; end

    % INIT MAP 
    map.logOdds = zeros(params.mapSize, params.mapSize);
    map.prob    = 0.5 * ones(params.mapSize, params.mapSize);

    % INIT PATH TRACKING 
    target_index = 1;
    e_int        = 0;
    N            = size(path, 1);
    traj         = [];

    % ── INIT VISUALIZATION ──────────────────────────────────────
    handles = setupPlot(map.prob, path, params.origin, params.scale);
    r       = rateControl(params.rate);

    % ── MAIN LOOP ───────────────────────────────────────────────
    frame = 0;
    while target_index <= N

        frame = frame + 1;

        % ── SENSORS ─────────────────────────────────────────────
        [pose.x, pose.y, theta] = tbot.readPose();
        pose.theta = normalizeAngle(theta);
        traj = [traj; pose.x, pose.y];

        [~, data] = tbot.readLidar();
        idx  = tbot.getInRangeLidarDataIdx(data);
        scan = data.Cartesian(idx, :)';
      
        scan = scanFilter(scan);
        
        % ── GEOMETRY (shared by both branches) ──────────────────
        worldPts  = scanToWorld(scan, pose);
        gridPts   = worldToGrid(worldPts, params);
        robotGrid = worldToGrid([pose.x; pose.y], params);

        % ── MAPPING BRANCH ──────────────────────────────────────
        map.logOdds = logOddsUpdate(map.logOdds, robotGrid, gridPts, params);
        map.prob    = 1 ./ (1 + exp(-map.logOdds));

        % ── TRACKING BRANCH ─────────────────────────────────────
        waypoint.x = path(target_index, 1);
        waypoint.y = path(target_index, 2);

        % avoidance reads the map.prob we just updated above
        params.map = map.prob;
        [target, h, alpha] = computeTarget(pose, waypoint, params, avoidance);

        distance = getEuclidianDistance(pose, waypoint);
        e        = getEuclidianDistance(pose, target) - params.d_star;
        phi      = getAngularError(pose, target);

        e_int            = e_int + e * params.dt;
        linear_velocity  = params.kv * e + params.ki * e_int;
        angular_velocity = params.ks * phi;
        
        linear_velocity = min(0.08, max(linear_velocity, 0));
        angular_velocity = min(1, max(angular_velocity, -1));
        tbot.setVelocity(linear_velocity, angular_velocity);

        % Advance waypoint
        if distance < 0.1
            target_index = target_index + 1;
            e_int = 0;
        end

        % ── VISUALIZATION ───────────────────────────────────────
        if mod(frame, params.plotSkip) == 0
            updatePlot(handles, traj, pose, target, h, alpha);
            % also refresh the map image
            set(handles.mapImg, 'CData', map.prob');
            drawnow limitrate;
        end

        waitfor(r);
    end

    tbot.setVelocity(0, 0);

    % ── SAVE ────────────────────────────────────────────────────
    binaryMap = map.prob > 0.5;
    imwrite(uint8(flipud(1 - map.prob) * 255), savePath + ".png");
    save(savePath + ".mat", 'map', 'binaryMap');
end


% computeTarget
function [target, h, alpha] = computeTarget(pose, waypoint, params, avoidance)

    switch avoidance
        case "vff"
            h = 0;
            alpha= 0;
            target = computeTargetVFF(pose, waypoint, params);
        case "vfh"
            [target, h, alpha] = computeTargetVFH(pose, waypoint, params);
        otherwise
            target = waypoint;
    end
end

% computeTargetVFF 
function target = computeTargetVFF(pose, waypoint, params)
    dist    = getEuclidianDistance(pose, waypoint);
    [Fa, Fr] = VFF([pose.x, pose.y], [waypoint.x, waypoint.y], ...
                   params.map, 10, params.scale, params.origin);
    F  = Fa + Fr;
    Fn = norm(F);

    if Fn < 1e-6
        target = pose;
    else
        dir_hat  = F / Fn;
        L        = min(1, dist);
        target.x = pose.x + L * dir_hat(1);
        target.y = pose.y + L * dir_hat(2);
    end
end

% computeTargetVFH 
function [target, h, alpha] = computeTargetVFH(pose, waypoint, params)
    [steerAngle, h, alpha] = VFH([pose.x, pose.y, pose.theta], [waypoint.x, waypoint.y], ...
                     params.map, 10);

    if isnan(steerAngle)
        % No free space — hold position
        target = pose;
        return;
    end

    dist     = getEuclidianDistance(pose, waypoint);
    L        = min(1, dist);
    target.x = pose.x + L * cos(steerAngle);
    target.y = pose.y + L * sin(steerAngle);
end