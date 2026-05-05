function mapBuildingWithTracking(tbot, params, pathWorld, savePath, avoidance)

    if nargin < 5, avoidance = "none"; end

    % Init map 
    map.logOdds = zeros(params.mapSize, params.mapSize);
    map.prob    = 0.5 * ones(params.mapSize, params.mapSize);

    % Give VFH a live reference — we'll update this pointer each tick
    params.map = map.prob;

    % Init path tracking state 
    N            = size(pathWorld, 1);
    target_index = 1;
    e_int        = 0;
    traj         = [];

    % Init plot 
    handles = setupPathTrackingPlot(map.prob, pathWorld, params.origin, params.scale);

    r = rateControl(params.rate);
    frame = 0;

    % Main loop 
    for t = 0 : params.dt : params.T

        frame = frame + 1;

        % 1. READ SENSORS
        [pose.x, pose.y, pose.theta, ~] = tbot.readPose();
        pose.theta = normalizeAngle(pose.theta);
        
        if mod(frame, params.plotSkip) == 0
            [~, data] = tbot.readLidar();
            idx  = tbot.getInRangeLidarDataIdx(data);
            scans = data.Cartesian(idx, :);
            scans = scanFilter(scans);

            % 2. UPDATE MAP
            worldPts  = scanToWorld(scans, pose);
            gridPts   = worldToGrid(worldPts, params);
            robotGrid = worldToGrid([pose.x, pose.y], params);
    
            map.logOdds = logOddsUpdate(map.logOdds, robotGrid, gridPts, params);
            map.prob    = 1 ./ (1 + exp(-map.logOdds));
    
            % Critical: push fresh map into params so VFH sees it
            params.map = map.prob;
        end
        
        % 3. PATH TRACKING + AVOIDANCE
        waypoint.x = pathWorld(target_index, 1);
        waypoint.y = pathWorld(target_index, 2);

        [target, h, alpha] = computeTarget(pose, waypoint, params, avoidance);

        distance = getEuclidianDistance(pose, waypoint);
        e        = getEuclidianDistance(pose, target) - params.d_star;
        phi      = getAngularError(pose, target);

        e_int           = e_int + e * params.dt;
        linear_velocity  = params.kv * e + params.ki * e_int;
        angular_velocity = params.ks * phi;
        
        linear_velocity = max(0, min(0.1, linear_velocity));
        angular_velocity = max(-0.8, min(0.8, angular_velocity));

        tbot.setVelocity(linear_velocity, angular_velocity);

        traj = [traj; pose.x, pose.y];

        % 4. ADVANCE WAYPOINT
        if distance < 0.3 && target_index < N
            target_index = target_index + 1;
            e_int = 0;
        end

        % 5. VISUALISE (every plotSkip frames)
        
        updatePathTrackingPlot(handles, traj, pose, target, h, alpha);
        set(handles.mapImage, 'CData', map.prob');
        

        % 6. GOAL CHECK
        if target_index >= N && distance < 0.3
            break;
        end

        waitfor(r);
    end

    tbot.setVelocity(0, 0);

    % SAVE
    binaryMap = map.prob > 0.5;
    if ~isempty(savePath)
        imwrite(uint8(flipud(1 - map.prob) * 255), savePath + ".png");
        save(savePath + ".mat", 'map', 'binaryMap');
    end
end


% computeTarget
function [target, h, alpha] = computeTarget(pose, waypoint, params, avoidance)
    
    h = 0;
    alpha= 0;
    switch avoidance
        case "vff"
            target = computeTargetVFF(pose, waypoint, params);
        case "vfh"
            [target, h, alpha] = computeTargetVFH(pose, waypoint, params);
        otherwise
            target = waypoint;
    end
end


% computeTargetVFF 
function target = computeTargetVFF(pose, waypoint, params)
    
    [Fa, Fr] = VFF([pose.x, pose.y], [waypoint.x, waypoint.y], ...
                   params.map, 10, params.scale, params.origin);
    F  = Fa + Fr;
    Fn = norm(F);

    if Fn < 1e-6
        target = pose;
    else
        dir_hat  = F / Fn;
        dist    = getEuclidianDistance(pose, waypoint);
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
        % No free space - hold position
        target = pose;
        return;
    end

    dist     = getEuclidianDistance(pose, waypoint);
    L        = min(1, dist);
    target.x = pose.x + L * cos(steerAngle);
    target.y = pose.y + L * sin(steerAngle);
end