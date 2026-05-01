function PathTracking2(tbot, params, path, handles, avoidance)

    if nargin < 5
        avoidance="none";
    end

    target_index = 1;
    e_int = 0;
    N = size(path, 1); % Number of waypoints
    
    traj = [];

    mapParams.scale   = 20;
    mapParams.origin  = 0;
    mapParams.mapSize = 80;
    
    map = zeros(mapParams.mapSize, mapParams.mapSize);

    r = rateControl(params.rate);   % adiciona antes do for
    
    for t= 0:params.dt:params.T

        [pose.x, pose.y, pose.theta, ~] = tbot.readPose();
        pose.theta = normalizeAngle(pose.theta);
        
        traj = [traj; [pose.x, pose.y]];
            
        % Waypoint actual
        waypoint.x = path(target_index, 1);
        waypoint.y = path(target_index, 2);
        
        map = bayesianMapUpdate(tbot, map, mapParams);

        % Target intermediário (VFF se map existir, directo caso contrário)
        [target, h, alpha] = computeTarget(pose, waypoint, params, avoidance, map);

        % Erros
        distance = getEuclidianDistance(pose, waypoint);
        e        = getEuclidianDistance(pose, target) - params.d_star;
        phi      = getAngularError(pose, target);

        % Controlo
        e_int            = e_int + e * params.dt;
        linear_velocity  = params.kv * e + params.ki * e_int;
        angular_velocity = params.ks * phi;

        tbot.setVelocity(linear_velocity, angular_velocity);

        % Visualização
        % updatePlot(handles, traj, pose, target);
        updatePlot(handles, traj, pose, target, h, alpha);

        % Avança waypoint
        if distance < 0.3 && target_index < N
            target_index = target_index + 1;
            e_int = 0;
        end

        if target_index >= N && distance < 0.1
            break;
        end

        waitfor(r);
    end

    tbot.setVelocity(0, 0);
end

% ── computeTarget ─────────────────────────────────────────────────────────────
function [target, h, alpha] = computeTarget(pose, waypoint, params, avoidance, map)

    switch avoidance
        case "vff"
            h = 0;
            alpha= 0;
            target = computeTargetVFF(pose, waypoint, params, map);
        case "vfh"
            [target, h, alpha] = computeTargetVFH(pose, waypoint, params, map);
        otherwise
            target = waypoint;
    end
end


% ── computeTargetVFF ──────────────────────────────────────────────────────────
function target = computeTargetVFF(pose, waypoint, params, map)
    dist    = getEuclidianDistance(pose, waypoint);
    [Fa, Fr] = VFF([pose.x, pose.y], [waypoint.x, waypoint.y], ...
                   map, 10, params.scale, params.origin);
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


% ── computeTargetVFH ──────────────────────────────────────────────────────────
function [target, h, alpha] = computeTargetVFH(pose, waypoint, params, map)
    [steerAngle, h, alpha] = VFH([pose.x, pose.y, pose.theta], [waypoint.x, waypoint.y], ...
                     map, 10);

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