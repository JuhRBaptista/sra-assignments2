function basicSLAM(tbot, params, path, avoidance, savePath)
    
    if nargin < 5
        avoidance="none";
    end
    
    target_index = 1;
    e_int = 0;
    N = size(path, 1); % Number of waypoints

    traj = [];

    % MAP INIT
    logOddsMap = zeros(params.mapSize, params.mapSize);
    probMap    = 0.5 * ones(params.mapSize, params.mapSize);

    % PLOT 
    handles = setupPlot(probMap);
    r = rateControl(params.rate);

    % ================= MAIN LOOP =================
    for t= 0:params.dt:params.T

        % --- Robot pose ---
        [pose.x, pose.y, pose.theta, ~] = tbot.readPose();
        theta = normalizeAngle(pose.theta);

        traj = [traj; [pose.x, pose.y]];

        % --- LIDAR ---
        [~, lddata] = tbot.readLidar();
        vidx  = tbot.getInRangeLidarDataIdx(lddata);
        scans = lddata.Cartesian(vidx, :);

        maxRange = 3.4; % LIDAR max range (assignment)
        minRange = 0.2; 
    
        xScans = scans(:,1);
        yScans = scans(:,2);
    
        % Filter invalid / too far or too close
        ranges = sqrt(xScans.^2 + yScans.^2);
        valid = ~isnan(ranges) & ~isinf(ranges) & (ranges < maxRange) & (ranges > minRange);
    
        xScans = xScans(valid);
        yScans = yScans(valid);
    
        % Transform to world
        xWorld = pose.x + xScans*cos(theta) - yScans*sin(theta);
        yWorld = pose.y + xScans*sin(theta) + yScans*cos(theta);
    
        % Convert to grid 
        xScaled = round(xWorld * params.scale) + params.origin;
        yScaled = round(yWorld * params.scale) + params.origin;
    
        xRobotGrid = round(pose.x * params.scale) + params.origin;
        yRobotGrid = round(pose.y * params.scale) + params.origin;

        logOddsMap = bayesianMapUpdate(xRobotGrid, yRobotGrid, xScaled, yScaled, logOddsMap, params);

        %Convert to probability 
        probMap = 1 ./ (1 + exp(-logOddsMap));
        
        % Update display 
        updatePlot(handles, probMap, yRobotGrid, xRobotGrid, theta);
        
        % Waypoint actual
        waypoint.x = path(target_index, 1);
        waypoint.y = path(target_index, 2);

        % Target intermediário 
        [target, h, alpha] = computeTarget(pose, waypoint, params, avoidance, probMap);

        % Erros
        distance = getEuclidianDistance(pose, waypoint);
        e        = getEuclidianDistance(pose, target) - params.d_star;
        phi      = getAngularError(pose, target);

        % Controlo
        e_int            = e_int + e * params.dt;
        linear_velocity  = params.kv * e + params.ki * e_int;
        angular_velocity = params.ks * phi;

        linear_velocity  = max(0, min(0.1, linear_velocity));
        angular_velocity = max(-1.84, min(1.84, angular_velocity));

        tbot.setVelocity(linear_velocity, angular_velocity);

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

    % SAVE 
    binaryMap = probMap > 0.5;

    imwrite(uint8(flipud(1 - probMap) * 255), savePath + ".png");
    save(savePath + ".mat", 'probMap', 'binaryMap', 'logOddsMap');

    close(handles.fig);
    tbot.stop();
end

function handles = setupPlot(map)
    handles.fig   = figure('KeyPressFcn', @keyboardHandle);
    handles.img   = imagesc(map);
    colormap(flipud(gray));
    set(gca, 'YDir', 'normal');
    axis equal;
    hold on;
    handles.robot = plot(0, 0, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    
    % Confirma que tudo ficou na mesma figura
    disp(handles.img.Parent.Parent == handles.fig);
end

function updatePlot(handles, map, xR, yR, theta)
    arrowLength = 10;
    set(handles.img,   'CData', map);
    set(handles.robot, 'XData', xR,  'YData', yR);
    drawnow limitrate;
end

% computeTarget
function [target, h, alpha] = computeTarget(pose, waypoint, params, avoidance, probMap)

    switch avoidance
        case "vff"
            h = 0;
            alpha= 0;
            target = computeTargetVFF(pose, waypoint, params, probMap);
        case "vfh"
            [target, h, alpha] = computeTargetVFH(pose, waypoint, params, probMap);
        otherwise
            h = 0;
            alpha= 0;
            target = waypoint;
    end
end

% compute Target VFF 
function target = computeTargetVFF(pose, waypoint, params, probMap)
    dist    = getEuclidianDistance(pose, waypoint);
    [Fa, Fr] = VFF([pose.x, pose.y], [waypoint.x, waypoint.y], ...
                   probMap, 10, params.scale, params.origin);
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

% compute Target VFH 
function [target, h, alpha] = computeTargetVFH(pose, waypoint, params, probMap)
    [steerAngle, h, alpha] = VFH([pose.x, pose.y, pose.theta], [waypoint.x, waypoint.y], ...
                     probMap, 10);

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